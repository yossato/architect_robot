# #png で出力
# set terminal png

# #出力ファイル名の決定
# set output "idnetify.png"

#区切り文字の変更
set datafile separator ","

# ------ 実験時のステップ入力値 ------
wheel_diameter = 0.38725;
r = wheel_diameter*pi*(100.0/60.0);
print r

# ------ データから読み取り ------
k_max = 19.0
y_max = 2.46696
y0 = 2.25
# ------------------------

# ------ 変数の算出 ------
a1 = -(1.0/k_max)*2*log((y_max/y0)-1.0)
a0 = (pi**2/k_max**2) + (a1**2/4.0)
b0 = a0*y0/r
omega = sqrt(a0)
zeta = a1/(2.0*sqrt(a0))
K = b0/a0
omega_d = omega*sqrt(.01-zeta**2)
theta = atan(sqrt(1.0-zeta**2)/zeta)
# ------------------------
print a1


# 二次遅れ系のステップ応答式
y_identified(x) = y0*K*(1-(exp(-zeta*omega*x)/sqrt(1-zeta**2))*sin(omega_d*x+theta))

# ------ 出力範囲の設定 ------
# 範囲は、データより
set xrange [0:147]
set yrange [-0.5:3.0]
# ------------------------

#描画
set style line 1 pointtype 3
plot y_identified(x) title 'Identified',\
     "../15-10-13-04:13-step-right-identify.csv" u 1:2 title 'Raw'
pause -1
unset style line
