function p_right_foot(θ) 
    argout_0 = zeros(3)
    argout_1 = zeros(9)  
    w0 = -1.0000000000000000e-03
    w1 = -1.1850000000000001e-01
    w2 = 1.7944534747016405e-12
    w3 = θ[4]
    w4 = cos(w3)
    w5 = (w2*w4)
    w6 = -3.4069969068184491e-13
    w3 = sin(w3)
    w7 = (w6*w3)
    w5 = (w5+w7)
    w7 = (w1*w5)
    w8 = -4.0000000000000001e-03
    w6 = (w6*w4)
    w9 = (w2*w3)
    w6 = (w6-w9)
    w9 = (w8*w6)
    w10 = -4.4000000000332967e-02
    w9 = (w9+w10)
    w7 = (w7+w9)
    w0 = (w0+w7)
    w7 = 1.2000000000000000e-01
    w9 = -7.0710678118969306e-01
    w10 = θ[5]
    w11 = cos(w10)
    w12 = (w9*w11)
    w13 = -7.0710678118340220e-01
    w10 = sin(w10)
    w14 = (w13*w10)
    w12 = (w12-w14)
    w14 = (w5*w12)
    w15 = 1.4621637234313312e-13
    w16 = (w15*w11)
    w17 = -5.0428550224523860e-12
    w18 = (w17*w10)
    w16 = (w16-w18)
    w18 = (w6*w16)
    w19 = -7.0710678118340209e-01
    w20 = (w19*w11)
    w21 = 7.0710678118969295e-01
    w22 = (w21*w10)
    w20 = (w20-w22)
    w18 = (w18-w20)
    w14 = (w14+w18)
    w18 = (w7*w14)
    w22 = 4.4999999999999997e-03
    w23 = -3.4623415245780462e-12
    w24 = (w23*w5)
    w25 = -3.6692315852518651e-12
    w25 = (w6+w25)
    w24 = (w24+w25)
    w25 = (w22*w24)
    w18 = (w18+w25)
    w0 = (w0+w18)
    w18 = 4.4263078443409087e-01
    w25 = 4.8966386501092529e-12
    w26 = θ[6]
    w27 = cos(w26)
    w28 = (w25*w27)
    w26 = sin(w26)
    w28 = (w28-w26)
    w29 = (w14*w28)
    w9 = (w9*w10)
    w13 = (w13*w11)
    w9 = (w9+w13)
    w5 = (w5*w9)
    w15 = (w15*w10)
    w17 = (w17*w11)
    w15 = (w15+w17)
    w6 = (w6*w15)
    w19 = (w19*w10)
    w21 = (w21*w11)
    w19 = (w19+w21)
    w6 = (w6-w19)
    w5 = (w5+w6)
    w6 = (w25*w26)
    w6 = (w27+w6)
    w21 = (w5*w6)
    w29 = (w29+w21)
    w21 = (w18*w29)
    w11 = -4.7394703016674455e-01
    w10 = (w25*w27)
    w10 = (w10-w26)
    w5 = (w5*w10)
    w25 = (w25*w26)
    w25 = (w25+w27)
    w14 = (w14*w25)
    w5 = (w5-w14)
    w14 = (w11*w5)
    w21 = (w21+w14)
    w0 = (w0+w21)
    argout_0[1] = w0
    w0 = -9.0999999999999998e-02
    w21 = 3.6650116868123461e-01
    w14 = (w21*w4)
    w27 = 9.3041759084579290e-01
    w26 = (w27*w3)
    w14 = (w14+w26)
    w26 = (w1*w14)
    w17 = (w27*w4)
    w21 = (w21*w3)
    w17 = (w17-w21)
    w21 = (w8*w17)
    w13 = 1.4990786390114618e-14
    w21 = (w21+w13)
    w26 = (w26+w21)
    w0 = (w0+w26)
    w26 = (w14*w12)
    w21 = (w17*w16)
    w13 = 3.4069969068184491e-13
    w30 = (w13*w20)
    w21 = (w21+w30)
    w26 = (w26+w21)
    w21 = (w7*w26)
    w30 = (w23*w14)
    w31 = 1.2501060661353660e-24
    w31 = (w17+w31)
    w30 = (w30+w31)
    w31 = (w22*w30)
    w21 = (w21+w31)
    w0 = (w0+w21)
    w21 = (w26*w28)
    w14 = (w14*w9)
    w17 = (w17*w15)
    w13 = (w13*w19)
    w17 = (w17+w13)
    w14 = (w14+w17)
    w17 = (w14*w6)
    w21 = (w21+w17)
    w17 = (w18*w21)
    w14 = (w14*w10)
    w26 = (w26*w25)
    w14 = (w14-w26)
    w26 = (w11*w14)
    w17 = (w17+w26)
    w0 = (w0+w17)
    argout_0[2] = w0
    w0 = (w27*w4)
    w17 = -3.6650116868123461e-01
    w26 = (w17*w3)
    w0 = (w0+w26)
    w1 = (w1*w0)
    w17 = (w17*w4)
    w27 = (w27*w3)
    w17 = (w17-w27)
    w8 = (w8*w17)
    w27 = 7.8955952887469675e-14
    w8 = (w8+w27)
    w1 = (w1+w8)
    w12 = (w0*w12)
    w16 = (w17*w16)
    w20 = (w2*w20)
    w16 = (w16+w20)
    w12 = (w12+w16)
    w7 = (w7*w12)
    w23 = (w23*w0)
    w16 = 6.5842653676402182e-24
    w16 = (w17+w16)
    w23 = (w23+w16)
    w22 = (w22*w23)
    w7 = (w7+w22)
    w1 = (w1+w7)
    w28 = (w12*w28)
    w0 = (w0*w9)
    w17 = (w17*w15)
    w2 = (w2*w19)
    w17 = (w17+w2)
    w0 = (w0+w17)
    w6 = (w0*w6)
    w28 = (w28+w6)
    w18 = (w18*w28)
    w0 = (w0*w10)
    w12 = (w12*w25)
    w0 = (w0-w12)
    w11 = (w11*w0)
    w18 = (w18+w11)
    w1 = (w1+w18)
    argout_0[3] = w1
    w1 = -2.2495102554717672e-01
    w18 = (w1*w29)
    w11 = -9.7437007143347421e-01
    w12 = (w11*w5)
    w18 = (w18+w12)
    argout_1[1] = w18
    w18 = (w1*w21)
    w12 = (w11*w14)
    w18 = (w18+w12)
    argout_1[2] = w18
    w18 = (w1*w28)
    w11 = (w11*w0)
    w18 = (w18+w11)
    argout_1[3] = w18
    w18 = 9.7437007143347421e-01
    w29 = (w18*w29)
    w5 = (w1*w5)
    w29 = (w29+w5)
    argout_1[4] = w29
    w21 = (w18*w21)
    w14 = (w1*w14)
    w21 = (w21+w14)
    argout_1[5] = w21
    w18 = (w18*w28)
    w1 = (w1*w0)
    w18 = (w18+w1)
    argout_1[6] = w18
    argout_1[7] = w24
    argout_1[8] = w30
    argout_1[9] = w23 
    position = vec(argout_0)
    orientation = reshape(argout_1, 3, 3) 
    return position
      
end