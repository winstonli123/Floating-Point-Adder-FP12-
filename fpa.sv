// fpa.sv – FP12 4-way adder with pipelined align and 18-bit accumulator,
// saturating to ±1_11110_110000.
// TIMING OPTIMIZED: Split Stage 6 normalization into 6a/6b - MINIMAL CHANGE
//
// Format: 1|5|6 (sign|exp|frac), bias=15
// Policy: subnormals flushed to 0, overflow -> ±(1_11110_110000), rounding = truncation

// -------------------------
// 4-bit CLA (kept if needed elsewhere)
// -------------------------
module cla4(
  input  wire [3:0] a,b,
  input  wire       cin,
  output wire [3:0] sum,
  output wire       cout,
  output wire       P, G
);
  wire [3:0] p = a ^ b;
  wire [3:0] g = a & b;
  wire c1 = g[0] | (p[0] & cin);
  wire c2 = g[1] | (p[1] & g[0]) | (p[1] & p[0] & cin);
  wire c3 = g[2] | (p[2] & g[1]) | (p[2] & p[1] & g[0]) | (p[2] & p[1] & p[0] & cin);
  wire c4 = g[3] | (p[3] & g[2]) | (p[3] & p[2] & g[1])
                  | (p[3] & p[2] & p[1] & g[0]) | (p[3] & p[2] & p[1] & p[0] & cin);
  assign sum  = { p[3]^c3, p[2]^c2, p[1]^c1, p[0]^cin };
  assign cout = c4;
  assign P = &p;
  assign G = g[3] | (p[3]&g[2]) | (p[3]&p[2]&g[1]) | (p[3]&p[2]&p[1]&g[0]);
endmodule

// -------------------------
// 2-bit CLA
// -------------------------
module cla2(
  input  wire [1:0] a,b,
  input  wire       cin,
  output wire [1:0] sum,
  output wire       cout,
  output wire       P, G
);
  wire [1:0] p = a ^ b;
  wire [1:0] g = a & b;
  wire c1 = g[0] | (p[0] & cin);
  wire c2 = g[1] | (p[1] & g[0]) | (p[1] & p[0] & cin);
  assign sum  = { p[1]^c1, p[0]^cin };
  assign cout = c2;
  assign P = &p;
  assign G = g[1] | (p[1]&g[0]);
endmodule

// -------------------------
// 14-bit blocked CLA: 4+4+4+2 (not used, kept for completeness)
// -------------------------
module cla14(
  input  wire [13:0] a,b,
  input  wire        cin,
  output wire [13:0] sum,
  output wire        cout
);
  wire [3:0] s0; wire c4;  wire P0,G0;
  cla4 u0 (.a(a[3:0]),   .b(b[3:0]),   .cin(cin), .sum(s0), .cout(), .P(P0), .G(G0));
  assign c4 = G0 | (P0 & cin);

  wire [3:0] s1; wire c8;  wire P1,G1;
  cla4 u1 (.a(a[7:4]),   .b(b[7:4]),   .cin(c4),  .sum(s1), .cout(), .P(P1), .G(G1));
  assign c8 = G1 | (P1 & c4);

  wire [3:0] s2; wire c12; wire P2,G2;
  cla4 u2 (.a(a[11:8]),  .b(b[11:8]),  .cin(c8),  .sum(s2), .cout(), .P(P2), .G(G2));
  assign c12 = G2 | (P2 & c8);

  wire [1:0] s3; wire c14; wire P3,G3;
  cla2 u3 (.a(a[13:12]), .b(b[13:12]), .cin(c12), .sum(s3), .cout(c14), .P(P3), .G(G3));

  assign sum  = { s3, s2, s1, s0 };
  assign cout = c14;
endmodule

// -------------------------
// FP12 four-input adder
// -------------------------
module fpa(
  input  wire        clk,
  input  wire        reset,
  input  wire [11:0] A,
  input  wire [11:0] B,
  input  wire [11:0] C,
  input  wire [11:0] D,
  input  wire        pushin,
  output reg         pushout,
  output reg  [11:0] Z
);

  // ---------- Stage 1: input capture ----------
  reg [11:0] stage1_A, stage1_B, stage1_C, stage1_D;
  reg        stage1_valid;

  always @(posedge clk) begin
    if (reset) begin
      stage1_valid <= 1'b0;
      stage1_A <= 12'b0; stage1_B <= 12'b0;
      stage1_C <= 12'b0; stage1_D <= 12'b0;
    end else begin
      stage1_valid <= pushin;
      stage1_A <= A; stage1_B <= B;
      stage1_C <= C; stage1_D <= D;
    end
  end

  wire [4:0] exp_A = stage1_A[10:6];
  wire [4:0] exp_B = stage1_B[10:6];
  wire [4:0] exp_C = stage1_C[10:6];
  wire [4:0] exp_D = stage1_D[10:6];

  wire [4:0] max_AB  = (exp_A >= exp_B) ? exp_A : exp_B;
  wire [4:0] max_CD  = (exp_C >= exp_D) ? exp_C : exp_D;
  wire [4:0] max_exp = (max_AB >= max_CD) ? max_AB : max_CD;

  // ---------- Stage 2: pre-align decode (light) ----------
  // Decode sign/exp/frac, build mag14, compute shift amounts relative to max_exp

  wire       A_sign = stage1_A[11];
  wire [5:0] A_frac = stage1_A[5:0];

  wire       B_sign = stage1_B[11];
  wire [5:0] B_frac = stage1_B[5:0];

  wire       C_sign = stage1_C[11];
  wire [5:0] C_frac = stage1_C[5:0];

  wire       D_sign = stage1_D[11];
  wire [5:0] D_frac = stage1_D[5:0];

  // shift amounts; use 31 as sentinel for "exp==0 → subnormal → flushed"
  wire [4:0] A_sh = (exp_A == 5'd0) ? 5'd31 :
                    (exp_A > max_exp) ? 5'd0 : (max_exp - exp_A);
  wire [4:0] B_sh = (exp_B == 5'd0) ? 5'd31 :
                    (exp_B > max_exp) ? 5'd0 : (max_exp - exp_B);
  wire [4:0] C_sh = (exp_C == 5'd0) ? 5'd31 :
                    (exp_C > max_exp) ? 5'd0 : (max_exp - exp_C);
  wire [4:0] D_sh = (exp_D == 5'd0) ? 5'd31 :
                    (exp_D > max_exp) ? 5'd0 : (max_exp - exp_D);

  // magnitude with hidden 1 at bit10
  wire [13:0] A_mag14 = {3'b000, 1'b1, A_frac, 3'b000, 1'b0};
  wire [13:0] B_mag14 = {3'b000, 1'b1, B_frac, 3'b000, 1'b0};
  wire [13:0] C_mag14 = {3'b000, 1'b1, C_frac, 3'b000, 1'b0};
  wire [13:0] D_mag14 = {3'b000, 1'b1, D_frac, 3'b000, 1'b0};

  reg        stage2_A_sign, stage2_B_sign, stage2_C_sign, stage2_D_sign;
  reg [4:0]  stage2_A_sh,   stage2_B_sh,   stage2_C_sh,   stage2_D_sh;
  reg [13:0] stage2_A_mag,  stage2_B_mag,  stage2_C_mag,  stage2_D_mag;
  reg [4:0]  stage2_max_exp;
  reg        stage2_valid;

  always @(posedge clk) begin
    if (reset) begin
      stage2_valid   <= 1'b0;
      stage2_max_exp <= 5'd0;

      stage2_A_sign <= 1'b0; stage2_B_sign <= 1'b0;
      stage2_C_sign <= 1'b0; stage2_D_sign <= 1'b0;

      stage2_A_sh   <= 5'd0; stage2_B_sh   <= 5'd0;
      stage2_C_sh   <= 5'd0; stage2_D_sh   <= 5'd0;

      stage2_A_mag  <= 14'd0; stage2_B_mag <= 14'd0;
      stage2_C_mag  <= 14'd0; stage2_D_mag <= 14'd0;
    end else begin
      stage2_valid   <= stage1_valid;
      stage2_max_exp <= max_exp;

      stage2_A_sign <= A_sign;
      stage2_B_sign <= B_sign;
      stage2_C_sign <= C_sign;
      stage2_D_sign <= D_sign;

      stage2_A_sh   <= A_sh;
      stage2_B_sh   <= B_sh;
      stage2_C_sh   <= C_sh;
      stage2_D_sh   <= D_sh;

      stage2_A_mag  <= A_mag14;
      stage2_B_mag  <= B_mag14;
      stage2_C_mag  <= C_mag14;
      stage2_D_mag  <= D_mag14;
    end
  end

  // ---------- Stage 3: final align, sticky, sign conversion ----------

  function automatic [14:0] do_align_from_pre;
    input        sign_in;
    input [4:0]  sh_in;
    input [13:0] mag_in;
    // returns {sticky, signed[13:0]}
    logic        sticky_local;
    logic [13:0] tmag14, sticky_mask;
    logic signed [13:0] shv_keep14;
    begin
      // Sentinel for subnormal: treat as zero
      if (sh_in == 5'd31) begin
        do_align_from_pre = 15'd0;
      end else begin
        // sticky from dropped magnitude bits
        if (sh_in == 0) begin
          sticky_local = 1'b0;
        end else if (sh_in >= 14) begin
          sticky_local = |mag_in;
        end else begin
          sticky_mask  = (14'd1 << sh_in) - 14'd1;
          sticky_local = |(mag_in & sticky_mask);
        end

        // logical right shift
        if (sh_in >= 14) tmag14 = 14'd0;
        else             tmag14 = mag_in >> sh_in;

        // if shift >= 11, hidden-1 and payload out: contribution 0
        if (sh_in >= 11) begin
          shv_keep14 = 14'sd0;
        end else begin
          shv_keep14 = sign_in ? -$signed({1'b0, tmag14})
                               :  $signed({1'b0, tmag14});
        end

        do_align_from_pre = { sticky_local, shv_keep14 };
      end
    end
  endfunction

  wire [14:0] A3 = do_align_from_pre(stage2_A_sign, stage2_A_sh, stage2_A_mag);
  wire [14:0] B3 = do_align_from_pre(stage2_B_sign, stage2_B_sh, stage2_B_mag);
  wire [14:0] C3 = do_align_from_pre(stage2_C_sign, stage2_C_sh, stage2_C_mag);
  wire [14:0] D3 = do_align_from_pre(stage2_D_sign, stage2_D_sh, stage2_D_mag);

  wire [13:0] A3_val = A3[13:0];
  wire [13:0] B3_val = B3[13:0];
  wire [13:0] C3_val = C3[13:0];
  wire [13:0] D3_val = D3[13:0];

  wire A3_stk = A3[14];
  wire B3_stk = B3[14];
  wire C3_stk = C3[14];
  wire D3_stk = D3[14];

  reg  [13:0] stage3_A_aligned, stage3_B_aligned, stage3_C_aligned, stage3_D_aligned;
  reg         stage3_valid;
  reg  [4:0]  stage3_max_exp;
  reg         stage3_sticky;

  always @(posedge clk) begin
    if (reset) begin
      stage3_valid     <= 1'b0;
      stage3_max_exp   <= 5'd0;
      stage3_A_aligned <= 14'd0; stage3_B_aligned <= 14'd0;
      stage3_C_aligned <= 14'd0; stage3_D_aligned <= 14'd0;
      stage3_sticky    <= 1'b0;
    end else begin
      stage3_valid     <= stage2_valid;
      stage3_max_exp   <= stage2_max_exp;
      stage3_A_aligned <= A3_val;
      stage3_B_aligned <= B3_val;
      stage3_C_aligned <= C3_val;
      stage3_D_aligned <= D3_val;
      stage3_sticky    <= (A3_stk | B3_stk | C3_stk | D3_stk);
    end
  end

  // ---------- Stage 4: 18-bit partial sums (A+B, C+D) ----------
  wire signed [17:0] A18 = {{4{stage3_A_aligned[13]}}, stage3_A_aligned};
  wire signed [17:0] B18 = {{4{stage3_B_aligned[13]}}, stage3_B_aligned};
  wire signed [17:0] C18 = {{4{stage3_C_aligned[13]}}, stage3_C_aligned};
  wire signed [17:0] D18 = {{4{stage3_D_aligned[13]}}, stage3_D_aligned};

  wire signed [17:0] sAB18 = A18 + B18;
  wire signed [17:0] sCD18 = C18 + D18;

  reg  signed [17:0] stage4_AB, stage4_CD;
  reg         [4:0]  stage4_max_exp;
  reg                stage4_valid;
  reg                stage4_sticky;

  always @(posedge clk) begin
    if (reset) begin
      stage4_valid   <= 1'b0;
      stage4_AB      <= 18'd0;
      stage4_CD      <= 18'd0;
      stage4_max_exp <= 5'd0;
      stage4_sticky  <= 1'b0;
    end else begin
      stage4_valid   <= stage3_valid;
      stage4_AB      <= sAB18;
      stage4_CD      <= sCD18;
      stage4_max_exp <= stage3_max_exp;
      stage4_sticky  <= stage3_sticky;
    end
  end

  // ---------- Stage 5: final 18-bit sum (AB + CD) ----------
  wire signed [17:0] final18 = stage4_AB + stage4_CD;

  reg  signed [17:0] stage5_sum18;
  reg         [4:0]  stage5_max_exp;
  reg                stage5_valid;
  reg                stage5_sticky;

  always @(posedge clk) begin
    if (reset) begin
      stage5_valid   <= 1'b0;
      stage5_sum18   <= 18'd0;
      stage5_max_exp <= 5'd0;
      stage5_sticky  <= 1'b0;
    end else begin
      stage5_valid   <= stage4_valid;
      stage5_sum18   <= final18;
      stage5_max_exp <= stage4_max_exp;
      stage5_sticky  <= stage4_sticky;
    end
  end

  // ---------- Normalization helpers ----------
  function automatic [4:0] lead1_17; input [16:0] v;
    begin
      if (v[16]) lead1_17=5'd16; else if (v[15]) lead1_17=5'd15; else if (v[14]) lead1_17=5'd14;
      else if (v[13]) lead1_17=5'd13; else if (v[12]) lead1_17=5'd12; else if (v[11]) lead1_17=5'd11;
      else if (v[10]) lead1_17=5'd10; else if (v[9])  lead1_17=5'd9;  else if (v[8])  lead1_17=5'd8;
      else if (v[7])  lead1_17=5'd7;  else if (v[6])  lead1_17=5'd6;  else if (v[5])  lead1_17=5'd5;
      else if (v[4])  lead1_17=5'd4;  else if (v[3])  lead1_17=5'd3;  else if (v[2])  lead1_17=5'd2;
      else if (v[1])  lead1_17=5'd1;  else if (v[0])  lead1_17=5'd0;  else lead1_17=5'd17;
    end
  endfunction

  // ---------- Stage 6a: abs value and leading-one (TIMING SPLIT) ----------
  reg         stage6a_sign;
  reg [16:0]  stage6a_mag17;
  reg [4:0]   stage6a_max_exp;
  reg         stage6a_sticky;
  reg         stage6a_valid;

  always @(posedge clk) begin
    if (reset) begin
      stage6a_valid <= 1'b0;
      stage6a_sign <= 1'b0;
      stage6a_mag17 <= 17'd0;
      stage6a_max_exp <= 5'd0;
      stage6a_sticky <= 1'b0;
    end else begin
      stage6a_valid <= stage5_valid;
      stage6a_sign  <= stage5_sum18[17];
      stage6a_max_exp <= stage5_max_exp;
      stage6a_sticky <= stage5_sticky;
      
      // Absolute value computation only
      stage6a_mag17 <= stage5_sum18[17] ? (~stage5_sum18[16:0] + 17'd1) 
                                        : stage5_sum18[16:0];
    end
  end

  // ---------- Stage 6b: normalize (same logic as original) ----------
  function automatic [11:0] normalize_from_abs;
    input        sign;
    input [16:0] mag17;
    input [4:0]  max_exp;
    input        sticky_in;

    logic [4:0]  lead_one_pos;
    logic signed [6:0] exp_adjust, new_exp_calc;
    logic [4:0]  new_exp;
    logic [5:0]  new_frac;

    integer s, ls;
    logic [16:0] m, t, mask;
    logic drop_or;
    logic G,R,S;

    begin
      if (mag17 == 17'b0) begin
        normalize_from_abs = 12'b0;
      end else begin
        lead_one_pos = lead1_17(mag17);

        // Hidden-1 target lane = bit10
        exp_adjust   = $signed({2'b00, lead_one_pos}) - 7'd10;
        new_exp_calc = $signed({2'b00, max_exp}) + exp_adjust;

        m = mag17;
        s = lead_one_pos - 10;

        if (s >= 0) begin
          if (s >= 17) begin
            t = 17'b0; drop_or = |m;
          end else begin
            t = m >> s;
            mask    = (s==0) ? 17'b0 : ((17'h1<<s) - 17'h1);
            drop_or = (s==0) ? 1'b0  : |(m & mask);
          end
          new_frac = t[9:4];
          G = t[3]; R = t[2]; S = t[1] | drop_or | sticky_in;
        end else begin
          ls = -s;
          t  = (ls >= 17) ? 17'b0 : (m << ls);
          new_frac = t[9:4];
          G = 1'b0; R = 1'b0; S = sticky_in;
        end

        // Saturation / underflow policy
        if (new_exp_calc < 7'd2 || new_exp_calc[6]) begin
          normalize_from_abs = 12'b0;
        end else if (new_exp_calc > 7'd30) begin
          normalize_from_abs = {sign, 5'b11110, 6'b110000};
        end else begin
          new_exp = new_exp_calc[4:0];

          if (new_exp == 5'b11110 && new_frac > 6'b110000) begin
            normalize_from_abs = {sign, 5'b11110, 6'b110000};
          end else begin
            normalize_from_abs = {sign, new_exp, new_frac};
          end
        end
      end
    end
  endfunction

  reg [11:0] stage6b_result;
  reg        stage6b_valid;

  always @(posedge clk) begin
    if (reset) begin
      stage6b_valid  <= 1'b0;
      stage6b_result <= 12'b0;
    end else begin
      stage6b_valid  <= stage6a_valid;
      stage6b_result <= stage6a_valid
                       ? normalize_from_abs(stage6a_sign, stage6a_mag17, 
                                           stage6a_max_exp, stage6a_sticky)
                       : 12'b0;
    end
  end

  // ---------- Stage 7: output ----------
  always @(posedge clk) begin
    if (reset) begin
      pushout <= 1'b0;
      Z       <= 12'b0;
    end else begin
      pushout <= stage6b_valid;
      Z       <= stage6b_result;
    end
  end
endmodule
