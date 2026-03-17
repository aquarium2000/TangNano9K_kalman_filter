// =============================================================================
// Project: Tang Nano 9K Kalman Filter Implementation
// Design:  Basic Kalman Filter with Fixed Gain (K=0.5)
// Author:  Makoto Kawamura
// Date:    2026-03-18
// Description: 
//   - 固定小数点演算を用いた簡易カルマンフィルタの実装
//   - LFSRによるノイズ生成と、それを用いた擬似観測データの平滑化
//   - 処理結果をUART経由でPCへ送信 (115200bps想定)
// =============================================================================

module top(
    input  clk,      // システムクロック: 27MHz (Pin 52)
    output uart_tx,  // UART送信信号 (Pin 17)
    output [5:0] led // ステータス表示用LED (Pin 10-16)
);

    // 27MHz / 5,400,000 = 5Hz (0.2秒間隔)
    parameter UPDATE_INTERVAL = 25'd5_400_000; 

    // --- カルマンフィルタ変数 (Q16.16 固定小数点形式) ---
    reg signed [31:0] x = 32'h0000_0000;  // 状態推定値 (State Estimate)
    reg signed [31:0] p = 32'h0001_0000;  // 推定誤差共分散 (Error Covariance)
    wire signed [31:0] q = 32'h0000_0100; // プロセスノイズ共分散 (Process Noise)
    wire signed [31:0] r = 32'h0001_0000; // 観測ノイズ共分散 (Measurement Noise)
    reg signed [31:0] z;                  // 観測値 (Measurement)

    // --- 正規分布ノイズ生成 (中央極限定理を利用) ---
    // 4つの16bit LFSRを用いて一様乱数を生成し、加算することで正規分布を近似
    reg [15:0] l1=16'hACE1, l2=16'h1234, l3=16'h5678, l4=16'h9ABC;
    always @(posedge clk) begin
        l1 <= {l1[14:0], l1[15]^l1[13]^l1[12]^l1[10]};
        l2 <= {l2[14:0], l2[15]^l2[14]^l2[12]^l2[3]};
        l3 <= {l3[14:0], l3[15]^l3[11]^l3[9]^l3[5]};
        l4 <= {l4[14:0], l4[15]^l4[7]^l4[4]^l4[2]};
    end
    
    // 生成したノイズをスケーリング (>>> 8)
    wire signed [31:0] g_noise = ($signed({{16{l1[15]}},l1}) + $signed({{16{l2[15]}},l2}) + 
                                   $signed({{16{l3[15]}},l3}) + $signed({{16{l4[15]}},l4})) >>> 8;

    // --- 制御・通信用レジスタ ---
    reg [24:0] timer = 0;
    reg [4:0]  step = 0;
    reg [31:0] z_latch, x_latch; // 送信中に値が変わらないよう保持するデータバッファ
    reg [3:0]  char_count = 0;   // 16進数文字の送信カウンタ
    reg [7:0]  uart_char;        // 送信対象のアスキーコード
    reg        uart_start = 0;
    wire       uart_busy;

    reg signed [63:0] m_tmp;                    // 乗算結果の一時保持用 (Q16.16 * Q16.16 = Q32.32)
    reg signed [31:0] k_gain = 32'h0000_8000;   // カルマンゲイン K=0.5 (固定値)

    // --- UART送信データ選択ロジック ---
    // 現在のステップに応じて、観測値(z)か推定値(x)を選択
    wire [31:0] active_val = (step <= 8) ? z_latch : x_latch;
    // 32bit値を4bitずつ取り出し、16進数文字に変換
    wire [3:0]  current_hex = active_val[(4'd7 - char_count) * 4 +: 4];

    // --- 統合メイン・ステートマシン ---
    always @(posedge clk) begin
        uart_start <= 1'b0; // パルス状にスタート信号を供給

        case (step)
            // [Step 0] 更新タイミング待ち
            0: begin 
                if (timer >= UPDATE_INTERVAL) begin
                    timer <= 0;
                    z <= 32'h0001_0000 + g_noise; // 観測値の生成 (1.0 + ノイズ)
                    step <= 1;
                end else timer <= timer + 1;
            end
            
            // --- カルマンフィルタ演算ステップ ---
            // [Step 1] 予測更新: P = P + Q
            1: begin p <= p + q; step <= 2; end
            
            // [Step 2-3] 推定更新: x = x + K * (z - x)
            2: begin m_tmp <= k_gain * (z - x); step <= 3; end
            3: begin x <= x + m_tmp[47:16]; step <= 4; end // 小数点位置調整 (Q32.32 -> Q16.16)
            
            // [Step 4-5] 推定誤差共分散更新: P = (1 - K) * P
            4: begin m_tmp <= (32'h0001_0000 - k_gain) * p; step <= 5; end
            5: begin
                p <= m_tmp[47:16];
                z_latch <= z;       // 送信用バッファへコピー
                x_latch <= x;       // 送信用バッファへコピー
                char_count <= 0;
                step <= 6;
            end

            // --- UART送信プロセス: 観測値 z (8文字の16進数) ---
            6: if (!uart_busy) begin
                // ASCII変換 (0-9 は 0x30, A-F は 0x37 を加算)
                uart_char <= (current_hex < 10) ? (8'h30 + {4'b0, current_hex}) : (8'h37 + {4'b0, current_hex});
                uart_start <= 1'b1;
                step <= 7;
            end
            7: if (uart_busy) step <= 8;
            8: if (!uart_busy) begin
                if (char_count == 7) begin char_count <= 0; step <= 9; end
                else begin char_count <= char_count + 1; step <= 6; end
            end

            // --- 送信プロセス: 区切り文字 (スペース) ---
            9: if (!uart_busy) begin
                uart_char <= 8'h20; // Space
                uart_start <= 1'b1;
                step <= 10;
            end
            10: if (uart_busy) step <= 11;

            // --- UART送信プロセス: 推定値 x (8文字の16進数) ---
            11: if (!uart_busy) begin
                uart_char <= (current_hex < 10) ? (8'h30 + {4'b0, current_hex}) : (8'h37 + {4'b0, current_hex});
                uart_start <= 1'b1;
                step <= 12;
            end
            12: if (uart_busy) step <= 13;
            13: if (!uart_busy) begin
                if (char_count == 7) begin char_count <= 0; step <= 14; end
                else begin char_count <= char_count + 1; step <= 11; end
            end

            // --- 送信プロセス: 改行 (CR + LF) ---
            14: if (!uart_busy) begin uart_char <= 8'h0D; uart_start <= 1'b1; step <= 15; end
            15: if (uart_busy) step <= 16;
            16: if (!uart_busy) begin uart_char <= 8'h0A; uart_start <= 1'b1; step <= 17; end
            17: if (uart_busy) step <= 18;
            18: if (!uart_busy) step <= 0; // サイクル完了、待機状態へ

            default: step <= 0;
        endcase
    end

    // --- 下位モジュール・インスタンス ---
    // UART送信コア
    uart_tx_core u_core (
        .clk(clk), 
        .start(uart_start), 
        .data(uart_char), 
        .tx(uart_tx), 
        .busy(uart_busy)
    );

    // LED表示: 推定値の上位ビットを反転表示 (負論理LED想定)
    assign led = ~x[21:16];

endmodule

// -----------------------------------------------------------------------------
// Module: uart_tx_core
// Description: 標準的なUART送信(8-N-1)を行うシンプルな回路
// -----------------------------------------------------------------------------
module uart_tx_core #(
    parameter [15:0] BIT_PERIOD = 16'd234 // 27MHz / 115200bps ≒ 234
) (
    input clk,
    input start,
    input [7:0] data,
    output reg tx,
    output busy
);
    reg [2:0] s = 0;     // 状態
    reg [15:0] t = 0;    // ボーレートタイマー
    reg [2:0] b_c = 0;   // ビットカウンタ
    reg [7:0] d_r;       // データレジスタ

    assign busy = (s != 0);

    always @(posedge clk) begin
        case (s)
            0: begin // IDLE
                tx <= 1;
                t <= 0;
                if (start) begin d_r <= data; s <= 1; end
            end
            1: begin // START BIT
                tx <= 0;
                if (t >= BIT_PERIOD-1) begin t <= 0; s <= 2; end else t <= t + 1;
            end
            2: begin // DATA BITS
                tx <= d_r[b_c];
                if (t >= BIT_PERIOD-1) begin 
                    t <= 0; 
                    if (b_c == 7) s <= 3; else b_c <= b_c + 1;
                end else t <= t + 1;
            end
            3: begin // STOP BIT
                tx <= 1;
                if (t >= BIT_PERIOD-1) begin t <= 0; b_c <= 0; s <= 0; end else t <= t + 1;
            end
        endcase
    end
endmodule