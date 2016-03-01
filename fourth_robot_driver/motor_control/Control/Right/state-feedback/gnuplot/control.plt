# #png で出力
# set terminal png

# #出力ファイル名の決定
# set output "Control.png"

#区切り文字の変更
set datafile separator ","

# ------ データから読み取り ------
k_max = 19.0
y_max = 2.46696
y0 = 2.25
# ------------------------

# ------ 変数の算出 ------
a1 = -(1/k_max)*2*log((y_max/y0)-1)
a0 = (pi**2/k_max**2) + (a1**2/4)
b0 = a0*y0/y0
omega = sqrt(a0)
zeta = a1/(2*sqrt(a0))
K = b0/a0
omega_d = omega*sqrt(1-zeta**2)
theta = atan(sqrt(1-zeta**2)/zeta)
# ------------------------

# 二次遅れ系のステップ応答式
y_identified(x) = y0*K*(1-(exp(-zeta*omega*x)/sqrt(1-zeta**2))*sin(omega_d*x+theta))

# 入力パラメータの決定
# 極配置により決定
ramda2 = -0.15
# f1 = (-(ramda2+a1)*ramda2-a0)/(b0)
f1 = (-(ramda2+a1)*ramda2-a0)/(b0)

# 目標値の決定
r = 2.25

# 状態フィードバックを行った時の応答式
y_controled(x) = -(b0*r*exp(-1.0/2.0*x*(sqrt(-4.0*a0+a1**2-4.0*b0*f1)+a1))*(a1*(exp(x*sqrt(-4.0*a0+a1**2-4.0*b0*f1))-1)+sqrt(-4.0*a0+a1**2-4.0*b0*f1)*(exp(x*sqrt(-4.0*a0+a1**2-4.0*b0*f1))-2*exp(1.0/2.0*x*(sqrt(-4.0*a0+a1**2-4.0*b0*f1)+a1))+1)))/(2*(a0+b0*f1)*sqrt(-4.0*a0+a1**2-4.0*b0*f1));


u(x) = r - f1*y_controled(x)
print a0
print a1
print f1

# ------ 出力範囲の設定 ------
# 範囲は、データより
set xrange [0:147]
# set yrange [-0.5:3.0]
# ------------------------

# 描画
set style line 1 pointtype 3
plot y_identified(x) title 'Identified',\
     y_controled(x) title 'Controled',\
     u(x) title 'Input',\
     r title 'Target'
pause -1
# "../15-10-13-04:13-step-right-identify.csv" u 1:2 title 'Raw'
unset style line
