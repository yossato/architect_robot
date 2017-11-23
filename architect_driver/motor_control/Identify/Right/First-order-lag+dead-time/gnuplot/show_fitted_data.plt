# #png で出力
# set terminal png

# #出力ファイル名の決定
# set output "Identify.png"

#区切り文字の変更
set datafile separator ","


#出力範囲の設定
# set xrange [0:130]
# set yrange [-0.5:2.3]

#描画
plot "../data/15-10-13-04:13-step-right-fitting.csv" u 1:15 title 'Identify',\
     "../data/15-10-13-04:13-step-right-fitting.csv" u 1:2 title 'Raw'
pause -1
