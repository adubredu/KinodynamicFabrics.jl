function Jp_right_wrist(q)
    argout_0 = cell(48,1) 
    w0 = 0.0000000000000000e+00
    argout_0[1] = w0
    argout_0[2] = w0
    argout_0[3] = w0
    argout_0[4] = w0
    argout_0[5] = w0
    argout_0[6] = w0
    argout_0[7] = w0
    argout_0[8] = w0
    argout_0[9] = w0
    argout_0[10] = w0
    argout_0[11] = w0
    argout_0[12] = w0
    argout_0[13] = w0
    argout_0[14] = w0
    argout_0[15] = w0
    argout_0[16] = w0
    argout_0[17] = w0
    argout_0[18] = w0
    argout_0[19] = w0
    argout_0[20] = w0
    argout_0[21] = w0
    argout_0[22] = w0
    argout_0[23] = w0
    argout_0[24] = w0
    w0 = -7.2237771320260419e-14
    argout_0[25] = w0
    w0 = -3.9999999999999919e-01
    argout_0[26] = w0
    w0 = -1.1999999999999993e-01
    argout_0[27] = w0
    w0 = -1.0000000000000000e+00
    argout_0[28] = w0
    w0 = -7.4468209376732375e-14
    argout_0[29] = w0
    w1 = 8.5020879225794488e-13
    argout_0[30] = w1
    w2 = 4.0000000000000002e-01
    w3 = -3.1700000000000001e-03
    w4 = 9.8480773537033595e-01
    w5 = q[5]
    w6 = cos(w5)
    w7 = (w4*w6)
    w8 = 1.7364827771892960e-01
    w5 = sin(w5)
    w9 = (w8*w5)
    w7 = (w7+w9)
    w9 = (w3*w7)
    w10 = 1.1055000000000001e-02
    w8 = (w8*w6)
    w4 = (w4*w5)
    w8 = (w8-w4)
    w4 = (w10*w8)
    w11 = 4.7186587970315941e-14
    w4 = (w4+w11)
    w9 = (w9+w4)
    w2 = (w2+w9)
    w9 = -2.7563737473217331e-01
    w4 = -1.7364827771892960e-01
    w11 = (w4*w6)
    w12 = 9.8480773537033583e-01
    w13 = (w12*w5)
    w11 = (w11+w13)
    w13 = (w9*w11)
    w14 = 9.6126169051447996e-01
    w12 = (w12*w6)
    w4 = (w4*w5)
    w12 = (w12-w4)
    w4 = (w14*w12)
    w15 = -2.5781783832324016e-25
    w4 = (w4+w15)
    w13 = (w13+w4)
    w4 = (w2*w13)
    w15 = -1.2000000000000000e-01
    w16 = (w3*w11)
    w17 = (w10*w12)
    w18 = -4.1329856204086466e-15
    w17 = (w17+w18)
    w16 = (w16+w17)
    w15 = (w15+w16)
    w16 = (w9*w7)
    w17 = (w14*w8)
    w18 = 2.9435244217359272e-24
    w17 = (w17+w18)
    w16 = (w16+w17)
    w17 = (w15*w16)
    w4 = (w4-w17)
    argout_0[31] = w4
    w4 = -1.0000000000000000e-03
    w17 = (w1*w6)
    w18 = 7.4315553710846416e-14
    w19 = (w18*w5)
    w17 = (w17+w19)
    w3 = (w3*w17)
    w18 = (w18*w6)
    w5 = (w1*w5)
    w18 = (w18-w5)
    w10 = (w10*w18)
    w5 = -5.5500000000000001e-02
    w10 = (w10+w5)
    w3 = (w3+w10)
    w4 = (w4+w3)
    w3 = (w4*w16)
    w9 = (w9*w17)
    w14 = (w14*w18)
    w10 = -3.4621194799910882e-12
    w14 = (w14+w10)
    w9 = (w9+w14)
    w14 = (w2*w9)
    w3 = (w3-w14)
    argout_0[32] = w3
    w3 = (w15*w9)
    w14 = (w4*w13)
    w3 = (w3-w14)
    argout_0[33] = w3
    w3 = (-w9)
    argout_0[34] = w3
    w3 = (-w13)
    argout_0[35] = w3
    w3 = (-w16)
    argout_0[36] = w3
    w3 = 1.6500000000000001e-01
    w14 = 6.7971465985726287e-01
    w10 = q[6]
    w5 = sin(w10)
    w6 = (w14*w5)
    w19 = -6.7971465985800317e-01
    w10 = cos(w10)
    w20 = (w19*w10)
    w6 = (w6+w20)
    w20 = (w11*w6)
    w21 = 1.9490505682401801e-01
    w22 = (w21*w5)
    w23 = -1.9490505681913683e-01
    w24 = (w23*w10)
    w22 = (w22+w24)
    w24 = (w12*w22)
    w25 = -7.0710678118623060e-01
    w26 = (w25*w5)
    w27 = -7.0710678118686476e-01
    w28 = (w27*w10)
    w26 = (w26+w28)
    w28 = (w0*w26)
    w24 = (w24+w28)
    w20 = (w20+w24)
    w24 = (w3*w20)
    w28 = -1.0000000000000001e-01
    w29 = (w28*w13)
    w24 = (w24+w29)
    w15 = (w15+w24)
    w24 = (w7*w6)
    w29 = (w8*w22)
    w30 = (w1*w26)
    w29 = (w29+w30)
    w24 = (w24+w29)
    w29 = 4.8966386501092529e-12
    w30 = (w29*w16)
    w30 = (w24+w30)
    w31 = (w15*w30)
    w32 = (w3*w24)
    w33 = (w28*w16)
    w32 = (w32+w33)
    w2 = (w2+w32)
    w32 = (w29*w13)
    w32 = (w20+w32)
    w33 = (w2*w32)
    w31 = (w31-w33)
    argout_0[37] = w31
    w6 = (w17*w6)
    w22 = (w18*w22)
    w22 = (w22-w26)
    w6 = (w6+w22)
    w22 = (w29*w9)
    w22 = (w6+w22)
    w26 = (w2*w22)
    w3 = (w3*w6)
    w28 = (w28*w9)
    w3 = (w3+w28)
    w4 = (w4+w3)
    w3 = (w4*w30)
    w26 = (w26-w3)
    argout_0[38] = w26
    w26 = (w4*w32)
    w3 = (w15*w22)
    w26 = (w26-w3)
    argout_0[39] = w26
    argout_0[40] = w22
    argout_0[41] = w32
    argout_0[42] = w30
    w26 = 3.8500000000000000e-02
    w3 = q[7]
    w28 = cos(w3)
    w31 = (w29*w28)
    w33 = (w20*w31)
    w34 = (w13*w28)
    w33 = (w33-w34)
    w14 = (w14*w10)
    w19 = (w19*w5)
    w14 = (w14-w19)
    w11 = (w11*w14)
    w21 = (w21*w10)
    w23 = (w23*w5)
    w21 = (w21-w23)
    w12 = (w12*w21)
    w25 = (w25*w10)
    w27 = (w27*w5)
    w25 = (w25-w27)
    w0 = (w0*w25)
    w12 = (w12+w0)
    w11 = (w11+w12)
    w3 = sin(w3)
    w12 = (w11*w3)
    w33 = (w33-w12)
    w12 = (w26*w33)
    w0 = 1.8500000000000000e-01
    w27 = (w0*w32)
    w12 = (w12+w27)
    w15 = (w15+w12)
    w12 = -1.8738621765379548e-12
    w7 = (w7*w14)
    w8 = (w8*w21)
    w1 = (w1*w25)
    w8 = (w8+w1)
    w7 = (w7+w8)
    w8 = (w7*w28)
    w29 = (w29*w3)
    w1 = (w24*w29)
    w27 = (w16*w3)
    w1 = (w1-w27)
    w8 = (w8+w1)
    w8 = (w12*w8)
    w24 = (w24*w31)
    w16 = (w16*w28)
    w24 = (w24-w16)
    w7 = (w7*w3)
    w24 = (w24-w7)
    w7 = 4.5239367807425879e-12
    w16 = (w7*w30)
    w16 = (w24+w16)
    w8 = (w8+w16)
    w16 = (w15*w8)
    w24 = (w26*w24)
    w30 = (w0*w30)
    w24 = (w24+w30)
    w2 = (w2+w24)
    w11 = (w11*w28)
    w20 = (w20*w29)
    w13 = (w13*w3)
    w20 = (w20-w13)
    w11 = (w11+w20)
    w11 = (w12*w11)
    w32 = (w7*w32)
    w33 = (w33+w32)
    w11 = (w11+w33)
    w33 = (w2*w11)
    w16 = (w16-w33)
    argout_0[43] = w16
    w17 = (w17*w14)
    w18 = (w18*w21)
    w18 = (w18-w25)
    w17 = (w17+w18)
    w18 = (w17*w28)
    w29 = (w6*w29)
    w25 = (w9*w3)
    w29 = (w29-w25)
    w18 = (w18+w29)
    w12 = (w12*w18)
    w6 = (w6*w31)
    w9 = (w9*w28)
    w6 = (w6-w9)
    w17 = (w17*w3)
    w6 = (w6-w17)
    w7 = (w7*w22)
    w7 = (w6+w7)
    w12 = (w12+w7)
    w2 = (w2*w12)
    w26 = (w26*w6)
    w0 = (w0*w22)
    w26 = (w26+w0)
    w4 = (w4+w26)
    w26 = (w4*w8)
    w2 = (w2-w26)
    argout_0[44] = w2
    w4 = (w4*w11)
    w15 = (w15*w12)
    w4 = (w4-w15)
    argout_0[45] = w4
    argout_0[46] = w12
    argout_0[47] = w11
    argout_0[48] = w8
    jacobian = reshape(argout_0, 6, 8)
    return jacobian
  end  