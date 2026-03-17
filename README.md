TangNano9K_Kalman_Filter_Series
Author: Makoto Kawamura

Tang Nano 9K (GOWIN GW1NR-9) を使用した、Verilog HDLによるカルマンフィルタの実装シリーズです。
基本的なアルゴリズムから、最終的には高速な除算器を用いたフルスペックの実装を目指します。

AI-Assisted Development:
本プロジェクトの開発プロセス（コード設計、最適化、デバッグ、ドキュメント作成）には、AIモデル Gemini をフルに活用しています。

プロジェクト概要
本リポジトリでは、FPGAのリソースを効率的に活用しながら、センサーデータのノイズ除去等に利用可能なカルマンフィルタを段階的に実装しています。

開発ステップ（子プロジェクト）

01_kalman_fixed_kgain_05
内容：ゲイン K=0.5 固定版。シフト演算のみで構成した軽量モデル。
ステータス：完了

02_kalman_dynamic_kgain
内容：ゲイン K を動的に演算。標準的な除算器の実装。
ステータス：開発中

03_kalman_fast_div_kgain
内容：除算の高速化（パイプライン等）によるスループット向上。
ステータス：計画中

開発環境

FPGA: Tang Nano 9K (GW1NR-LV9QN88PC6/I5)
IDE: GOWIN FPGA Designer (Gowin EDA)
AI Collaborator: Google Gemini

使用方法
各ディレクトリ内のプロジェクトファイルを Gowin EDA で開き、ビルド後に実機へ書き込んでください。シリアルモニタを通じて動作を確認できます。

(c) 2026 Makoto Kawamura