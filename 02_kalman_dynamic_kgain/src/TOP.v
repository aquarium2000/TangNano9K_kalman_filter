// =============================================================================
// Project: Tang Nano 9K Kalman Filter Implementation
// Design:  Advanced Kalman Filter with Dynamic Gain (K = P / (P + R))
// Author:  Makoto Kawamura
// Date:    2026-03-18
// Description: 
//   - 逐次比較型除算器を搭載し、カルマンゲインをリアルタイムに算出
//   - 誤差共分散 P の更新を含む完全なカルマン更新サイクルを実装
//   - Q16.16 固定小数点形式による高精度演算
// =============================================================================

module top(
    input  clk,      // 27MHz (Pin 52)
    output uart_tx,  // UART TX (Pin 17)
    output [5:0] led // ステータス表示LED (Pin 10-16)
);

    // 27MHz / 5,400,000 = 5Hz (0.2秒間隔)
    parameter UPDATE_INTERVAL = 25'd5_400_000; 

    // --- カルマンフィルタ変数 (Q16.16 固定小数点) ---
    reg signed [31:0] x = 32'h0000_0000;  // 状態推定値
    reg signed [31:0] p = 32'h0001_0000;  // 推定誤差共分散 (初期値 1.0)
    wire signed [31:0] q = 32'h0000_0100; // プロセスノイズ共分散 (システムの変化許容度)
    wire signed [31:0] r = 32'h0001_0000; // 観測ノイズ共分散 (1.0: センサーの信頼性)
    reg signed [31:0] z;                  // 観測値

    // --- 正規分布ノイズ生成 (LFSRによる近似) ---
    reg [15:0] l1=16'hACE1, l2=16'h1234, l3=16'h5678, l4=16'h9ABC;
    always @(posedge clk) begin
        l1 <= {l1[14:0], l1[15]^l1[13]^l1[12]^l1[10]};
        l2 <= {l2[14:0], l2[15]^l2[14]^l2[12]^l2[3]};
        l3 <= {l3[14:0], l3[15]^l3[11]^l3[9]^l3[5]};
        l4 <= {l4[14:0], l4[15]^l4[7]^l4[4]^l4[2]};
    end
    wire signed [31:0] g_noise = ($signed({{16{l1[15]}},l1}) + $signed({{16{l2[15]}},l2}) + 
                                   $signed({{16{l3[15]}},l3}) + $signed({{16{l4[15]}},l4})) >>> 8;

    // --- 演算・通信用信号 ---
    reg         div_start = 0; // 除算開始フラグ
    wire [31:0] div_k;         // 除算器からの出力 (K値)
    wire        div_busy;      // 除算実行中フラグ

    reg [24:0] timer = 0;
    reg [4:0]  step = 0;
    reg [31:0] z_latch, x_latch; // 送信データ保持用バッファ
    reg [3:0]  char_count = 0;
    reg [7:0]  uart_char;
    reg        uart_start = 0;
    wire       uart_busy;

    reg signed [63:0] m_tmp;   // 乗算結果一時保持 (Q32.32)
    reg signed [31:0] k_gain;  // 動的に算出されるカルマンゲイン

    // --- UART送信データ選択 ---
    wire [31:0] active_val = (step <= 11) ? z_latch : x_latch;
    wire [3:0]  current_hex = active_val[(4'd7 - char_count) * 4 +: 4];

    // --- 統合メイン・ステートマシン ---
    always @(posedge clk) begin
        uart_start <= 1'b0;
        div_start  <= 1'b0;

        case (step)
            // [Step 0] 更新待ち
            0: begin 
                if (timer >= UPDATE_INTERVAL) begin
                    timer <= 0;
                    z <= 32'h0001_0000 + g_noise; // ダミー観測値生成
                    step <= 1;
                end else timer <= timer + 1;
            end
            
            // --- カルマン演算フェーズ ---
            // [Step 1] 予測更新: P = P + Q
            1: begin p <= p + q; step <= 2; end 
            
            // [Step 2-4] ゲイン算出: K = P / (P + R)
            2: begin 
                div_start <= 1'b1; 
                step <= 3; 
            end
            3: if (div_busy) step <= 4;  // 除算開始待ち
            4: if (!div_busy) begin      // 除算完了待ち
                k_gain <= div_k;         // 算出された最適ゲインを格納
                step <= 5;
            end
            
            // [Step 5-6] 推定更新: x = x + K * (z - x)
            5: begin m_tmp <= k_gain * (z - x); step <= 6; end
            6: begin x <= x + m_tmp[47:16]; step <= 7; end
            
            // [Step 7-8] 誤差共分散更新: P = (1 - K) * P
            7: begin m_tmp <= (32'h0001_0000 - k_gain) * p; step <= 8; end
            8: begin
                p <= m_tmp[47:16];
                z_latch <= z;       // バッファへロック
                x_latch <= x;       // バッファへロック
                char_count <= 0;
                step <= 9;
            end

            // --- 送信フェーズ (UART) ---
            // [Step 9-11] 観測値 z 送信
            9: if (!uart_busy) begin
                uart_char <= (current_hex < 10) ? (8'h30 + {4'b0, current_hex}) : (8'h37 + {4'b0, current_hex});
                uart_start <= 1'b1;
                step <= 10;
            end
            10: if (uart_busy) step <= 11;
            11: if (!uart_busy) begin
                if (char_count == 7) begin char_count <= 0; step <= 12; end
                else begin char_count <= char_count + 1; step <= 9; end
            end

            // [Step 12-13] 区切り文字 (Space) 送信
            12: if (!uart_busy) begin uart_char <= 8'h20; uart_start <= 1'b1; step <= 13; end
            13: if (uart_busy) step <= 14;

            // [Step 14-16] 推定値 x 送信
            14: if (!uart_busy) begin
                uart_char <= (current_hex < 10) ? (8'h30 + {4'b0, current_hex}) : (8'h37 + {4'b0, current_hex});
                uart_start <= 1'b1;
                step <= 15;
            end
            15: if (uart_busy) step <= 16;
            16: if (!uart_busy) begin
                if (char_count == 7) begin char_count <= 0; step <= 17; end
                else begin char_count <= char_count + 1; step <= 14; end
            end

            // [Step 17-21] 改行送信 (CR + LF)
            17: if (!uart_busy) begin uart_char <= 8'h0D; uart_start <= 1'b1; step <= 18; end
            18: if (uart_busy) step <= 19;
            19: if (!uart_busy) begin uart_char <= 8'h0A; uart_start <= 1'b1; step <= 20; end
            20: if (uart_busy) step <= 21;
            21: if (!uart_busy) step <= 0;

            default: step <= 0;
        endcase
    end

    // --- 下位モジュール・インスタンス ---
    // Q16.16 固定小数点除算器
    divider_q16_16 div_inst(
        .clk(clk), 
        .start(div_start),
        .dividend(p), 
        .divisor(p + r),
        .quotient(div_k), 
        .busy(div_busy)
    );

    // UART送信コア
    uart_tx_core u_core (
        .clk(clk), 
        .start(uart_start), 
        .data(uart_char), 
        .tx(uart_tx), 
        .busy(uart_busy)
    );

endmodule

// -----------------------------------------------------------------------------
// Module: divider_q16_16
// Description: 32bit逐次比較型除算器。Q16.16出力を得るため分子を16bitシフトして演算。
// -----------------------------------------------------------------------------
module divider_q16_16(
    input clk, input start,
    input signed [31:0] dividend, divisor,
    output reg signed [31:0] quotient, output busy
);
    reg [63:0] q_dividend; // 演算用バッファ (分子をシフトするため64bit)
    reg [31:0] q_divisor;  // 分母保持
    reg [5:0]  q_count;    // 32ビット分カウント
    reg        q_busy = 0;
    assign busy = q_busy;

    always @(posedge clk) begin
        if (start && !q_busy) begin
            q_busy <= 1;
            q_count <= 0;
            // 固定小数点の除算(A/B)で精度を維持するため、Aを16bit左シフト
            q_dividend <= {32'd0, dividend} << 16;
            q_divisor  <= divisor;
        end else if (q_busy) begin
            if (q_count == 6'd32) begin
                q_busy <= 0;
                quotient <= q_dividend[31:0]; // 最終的な商を格納
            end else begin
                // 引き放ち法に近い逐次比較演算
                if (q_dividend[63:32] >= q_divisor) begin
                    q_dividend <= { (q_dividend[63:32] - q_divisor), q_dividend[31:0] } << 1;
                    q_dividend[0] <= 1'b1;
                end else begin
                    q_dividend <= q_dividend << 1;
                end
                q_count <= q_count + 6'd1;
            end
        end
    end
endmodule

// (uart_tx_core は実績のある共通コアのため、変更なし)
module uart_tx_core #(parameter [15:0] BIT_PERIOD = 16'd234) (
    input clk, input start, input [7:0] data, output reg tx, output busy
);
    reg [2:0] s = 0; reg [15:0] t = 0; reg [2:0] b_c = 0; reg [7:0] d_r;
    assign busy = (s != 0);
    always @(posedge clk) begin
        case (s)
            0: begin tx <= 1; t <= 0; if (start) begin d_r <= data; s <= 1; end end
            1: begin tx <= 0; if (t >= BIT_PERIOD-1) begin t <= 0; s <= 2; end else t <= t + 1; end
            2: begin tx <= d_r[b_c]; if (t >= BIT_PERIOD-1) begin t <= 0; if (b_c == 7) s <= 3; else b_c <= b_c + 1; end else t <= t + 1; end
            3: begin tx <= 1; if (t >= BIT_PERIOD-1) begin t <= 0; b_c <= 0; s <= 0; end else t <= t + 1; end
        endcase
    end
endmodule