function base_wrt_feet_helper!(p_output1, var1)
  t3 = cos(var1[3+1])
  t6 = cos(var1[5+1])
  t9 = sin(var1[3+1])
  t7 = sin(var1[4+1])
  t10 = sin(var1[5+1])
  t14 = -1.0 * t6*t9
  t15 = t3*t7*t10
  t16 = t14 + t15
  t8 = t3*t6*t7
  t11 = t9*t10
  t12 = t8 + t11
  t4 = cos(var1[4+1])
  t18 = cos(var1[6+1])
  t22 = sin(var1[6+1])
  t24 = cos(var1[7+1])
  t25 = -1.0 * t24
  t26 = 1.0 +  t25
  t28 = sin(var1[7+1])
  t32 = t18*t16
  t33 = -1.0 * t12*t22
  t34 = t32 + t33
  t40 = t18*t12
  t41 = t16*t22
  t42 = t40 + t41
  t55 = cos(var1[8+1])
  t56 = -1.0 * t55
  t57 = 1.0 +  t56
  t59 = sin(var1[8+1])
  t74 = -1.000000637725*t26
  t75 = 1.0 +  t74
  t76 = t3*t4*t75
  t77 = -0.930418*t34*t28
  t78 = -0.366501*t42*t28
  t79 = t76 + t77 + t78
  t48 = -0.340999127418*t26*t34
  t49 = -0.134322983001*t26
  t50 = 1.0 +  t49
  t51 = t50*t42
  t52 = 0.366501*t3*t4*t28
  t53 = t48 + t51 + t52
  t63 = -0.8656776547239999*t26
  t64 = 1.0 +  t63
  t65 = t64*t34
  t66 = -0.340999127418*t26*t42
  t67 = 0.930418*t3*t4*t28
  t68 = t65 + t66 + t67
  t84 = cos(var1[9+1])
  t85 = -1.0 * t84
  t86 = 1.0 +  t85
  t88 = sin(var1[9+1])
  t91 = -1.000000637725*t57
  t92 = 1.0 +  t91
  t93 = t92*t79
  t94 = -0.930418*t53*t59
  t95 = 0.366501*t68*t59
  t96 = t93 + t94 + t95
  t102 = 0.340999127418*t57*t53
  t103 = -0.134322983001*t57
  t104 = 1.0 +  t103
  t105 = t104*t68
  t106 = -0.366501*t79*t59
  t107 = t102 + t105 + t106
  t113 = -0.8656776547239999*t57
  t114 = 1.0 +  t113
  t115 = t114*t53
  t116 = 0.340999127418*t57*t68
  t117 = 0.930418*t79*t59
  t118 = t115 + t116 + t117
  t121 = cos(var1[10+1])
  t122 = -1.0 * t121
  t123 = 1.0 +  t122
  t125 = sin(var1[10+1])
  t128 = -0.930418*t88*t96
  t129 = 0.340999127418*t86*t107
  t130 = -0.8656776547239999*t86
  t131 = 1.0 +  t130
  t132 = t131*t118
  t133 = t128 + t129 + t132
  t139 = 0.366501*t88*t96
  t140 = -0.134322983001*t86
  t141 = 1.0 +  t140
  t142 = t141*t107
  t143 = 0.340999127418*t86*t118
  t144 = t139 + t142 + t143
  t149 = -1.000000637725*t86
  t150 = 1.0 +  t149
  t151 = t150*t96
  t152 = -0.366501*t88*t107
  t153 = 0.930418*t88*t118
  t154 = t151 + t152 + t153
  t156 = cos(var1[11+1])
  t157 = -1.0 * t156
  t158 = 1.0 +  t157
  t160 = sin(var1[11+1])
  t163 = 0.930418*t125*t133
  t164 = -0.366501*t125*t144
  t165 = -1.000000637725*t123
  t166 = 1.0 +  t165
  t167 = t166*t154
  t168 = t163 + t164 + t167
  t174 = -0.8656776547239999*t123
  t175 = 1.0 +  t174
  t176 = t175*t133
  t177 = 0.340999127418*t123*t144
  t178 = -0.930418*t125*t154
  t179 = t176 + t177 + t178
  t185 = 0.340999127418*t123*t133
  t186 = -0.134322983001*t123
  t187 = 1.0 +  t186
  t188 = t187*t144
  t189 = 0.366501*t125*t154
  t190 = t185 + t188 + t189
  t193 = cos(var1[12+1])
  t194 = -1.0 * t193
  t195 = 1.0 +  t194
  t197 = sin(var1[12+1])
  t200 = 0.366501*t160*t168
  t201 = 0.340999127418*t158*t179
  t202 = -0.134322983001*t158
  t203 = 1.0 +  t202
  t204 = t203*t190
  t205 = t200 + t201 + t204
  t211 = -0.930418*t160*t168
  t212 = -0.8656776547239999*t158
  t213 = 1.0 +  t212
  t214 = t213*t179
  t215 = 0.340999127418*t158*t190
  t216 = t211 + t214 + t215
  t221 = -1.000000637725*t158
  t222 = 1.0 +  t221
  t223 = t222*t168
  t224 = 0.930418*t160*t179
  t225 = -0.366501*t160*t190
  t226 = t223 + t224 + t225
  t19 = -1.0 * t18
  t20 = 1.0 +  t19
  t255 = t3*t6
  t256 = t9*t7*t10
  t257 = t255 + t256
  t251 = t6*t9*t7
  t252 = -1.0 * t3*t10
  t253 = t251 + t252
  t27 = -0.04500040093286238*t26
  t29 = -0.0846680539949003*t28
  t30 = t27 + t29
  t35 = 1.296332362046933e-7*var1[7+1]
  t36 = 0.07877668146182712*t26
  t37 = -0.04186915633414423*t28
  t38 = t35 + t36 + t37
  t43 = -3.2909349868922137e-7*var1[7+1]
  t44 = 0.03103092645718495*t26
  t45 = -0.016492681424499736*t28
  t46 = t43 + t44 + t45
  t262 = t18*t257
  t263 = -1.0 * t253*t22
  t264 = t262 + t263
  t266 = t18*t253
  t267 = t257*t22
  t268 = t266 + t267
  t54 = 1.296332362046933e-7*var1[8+1]
  t58 = -0.14128592423750855*t57
  t60 = -0.04186915633414423*t59
  t61 = t54 + t58 + t60
  t69 = 3.2909349868922137e-7*var1[8+1]
  t70 = 0.055653945343889656*t57
  t71 = 0.016492681424499736*t59
  t72 = t69 + t70 + t71
  t80 = -0.04500040093286238*t57
  t81 = 0.15185209683981668*t59
  t82 = t80 + t81
  t87 = 0.039853038461262744*t86
  t89 = -0.23670515095269612*t88
  t90 = t87 + t89
  t280 = t4*t75*t9
  t281 = -0.930418*t264*t28
  t282 = -0.366501*t268*t28
  t283 = t280 + t281 + t282
  t270 = -0.340999127418*t26*t264
  t271 = t50*t268
  t272 = 0.366501*t4*t9*t28
  t273 = t270 + t271 + t272
  t275 = t64*t264
  t276 = -0.340999127418*t26*t268
  t277 = 0.930418*t4*t9*t28
  t278 = t275 + t276 + t277
  t98 = -1.5981976069815686e-7*var1[9+1]
  t99 = 0.08675267452931407*t86
  t100 = 0.014606169134372047*t88
  t101 = t98 + t99 + t100
  t109 = -6.295460977284962e-8*var1[9+1]
  t110 = -0.22023473313910558*t86
  t111 = -0.03707996069223323*t88
  t112 = t109 + t110 + t111
  t120 = -1.6084556086870008e-7*var1[10+1]
  t124 = -0.29135406957765553*t123
  t126 = -0.02832985722118838*t125
  t127 = t120 + t124 + t126
  t285 = t92*t283
  t286 = -0.930418*t273*t59
  t287 = 0.366501*t278*t59
  t288 = t285 + t286 + t287
  t290 = 0.340999127418*t57*t273
  t291 = t104*t278
  t292 = -0.366501*t283*t59
  t293 = t290 + t291 + t292
  t295 = t114*t273
  t296 = 0.340999127418*t57*t278
  t297 = 0.930418*t283*t59
  t298 = t295 + t296 + t297
  t135 = -4.0833068682577724e-7*var1[10+1]
  t136 = 0.11476729583292707*t123
  t137 = 0.0111594154470601*t125
  t138 = t135 + t136 + t137
  t146 = 0.03044854601678662*t123
  t147 = -0.3131431996991197*t125
  t148 = t146 + t147
  t159 = -0.26285954081199375*t158
  t161 = -0.634735404786378*t160
  t162 = t159 + t161
  t300 = -0.930418*t88*t288
  t301 = 0.340999127418*t86*t293
  t302 = t131*t298
  t303 = t300 + t301 + t302
  t305 = 0.366501*t88*t288
  t306 = t141*t293
  t307 = 0.340999127418*t86*t298
  t308 = t305 + t306 + t307
  t310 = t150*t288
  t311 = -0.366501*t88*t293
  t312 = 0.930418*t88*t298
  t313 = t310 + t311 + t312
  t170 = 6.369237629068993e-8*var1[11+1]
  t171 = -0.5905692458505322*t158
  t172 = 0.24456909227538925*t160
  t173 = t170 + t171 + t172
  t181 = 1.6169269214444473e-7*var1[11+1]
  t182 = 0.2326311605896123*t158
  t183 = -0.09633822312984319*t160
  t184 = t181 + t182 + t183
  t192 = 1.7876586242383724e-7*var1[12+1]
  t196 = 0.3243041141817093*t195
  t198 = 0.02270383571304597*t197
  t199 = t192 + t196 + t198
  t315 = 0.930418*t125*t303
  t316 = -0.366501*t125*t308
  t317 = t166*t313
  t318 = t315 + t316 + t317
  t320 = t175*t303
  t321 = 0.340999127418*t123*t308
  t322 = -0.930418*t125*t313
  t323 = t320 + t321 + t322
  t325 = 0.340999127418*t123*t303
  t326 = t187*t308
  t327 = 0.366501*t125*t313
  t328 = t325 + t326 + t327
  t207 = 7.041766963257243e-8*var1[12+1]
  t208 = -0.8232948486053725*t195
  t209 = -0.05763710717422546*t197
  t210 = t207 + t208 + t209
  t218 = 0.06194758047549556*t195
  t219 = -0.8848655643005321*t197
  t220 = t218 + t219
  t330 = 0.366501*t160*t318
  t331 = 0.340999127418*t158*t323
  t332 = t203*t328
  t333 = t330 + t331 + t332
  t335 = -0.930418*t160*t318
  t336 = t213*t323
  t337 = 0.340999127418*t158*t328
  t338 = t335 + t336 + t337
  t230 = -1.000000637725*t195
  t231 = 1.0 +  t230
  t340 = t222*t318
  t341 = 0.930418*t160*t323
  t342 = -0.366501*t160*t328
  t343 = t340 + t341 + t342
  t236 = -0.8656776547239999*t195
  t237 = 1.0 +  t236
  t242 = -0.134322983001*t195
  t243 = 1.0 +  t242
  t367 = t4*t18*t10
  t368 = -1.0 * t4*t6*t22
  t369 = t367 + t368
  t371 = t4*t6*t18
  t372 = t4*t10*t22
  t373 = t371 + t372
  t385 = -1.0 * t75*t7
  t386 = -0.930418*t369*t28
  t387 = -0.366501*t373*t28
  t388 = t385 + t386 + t387
  t380 = t64*t369
  t381 = -0.340999127418*t26*t373
  t382 = -0.930418*t7*t28
  t383 = t380 + t381 + t382
  t375 = -0.340999127418*t26*t369
  t376 = t50*t373
  t377 = -0.366501*t7*t28
  t378 = t375 + t376 + t377
  t390 = t92*t388
  t391 = 0.366501*t383*t59
  t392 = -0.930418*t378*t59
  t393 = t390 + t391 + t392
  t395 = t104*t383
  t396 = 0.340999127418*t57*t378
  t397 = -0.366501*t388*t59
  t398 = t395 + t396 + t397
  t400 = 0.340999127418*t57*t383
  t401 = t114*t378
  t402 = 0.930418*t388*t59
  t403 = t400 + t401 + t402
  t405 = -0.930418*t88*t393
  t406 = 0.340999127418*t86*t398
  t407 = t131*t403
  t408 = t405 + t406 + t407
  t410 = 0.366501*t88*t393
  t411 = t141*t398
  t412 = 0.340999127418*t86*t403
  t413 = t410 + t411 + t412
  t415 = t150*t393
  t416 = -0.366501*t88*t398
  t417 = 0.930418*t88*t403
  t418 = t415 + t416 + t417
  t420 = 0.930418*t125*t408
  t421 = -0.366501*t125*t413
  t422 = t166*t418
  t423 = t420 + t421 + t422
  t425 = t175*t408
  t426 = 0.340999127418*t123*t413
  t427 = -0.930418*t125*t418
  t428 = t425 + t426 + t427
  t430 = 0.340999127418*t123*t408
  t431 = t187*t413
  t432 = 0.366501*t125*t418
  t433 = t430 + t431 + t432
  t435 = 0.366501*t160*t423
  t436 = 0.340999127418*t158*t428
  t437 = t203*t433
  t438 = t435 + t436 + t437
  t440 = -0.930418*t160*t423
  t441 = t213*t428
  t442 = 0.340999127418*t158*t433
  t443 = t440 + t441 + t442
  t445 = t222*t423
  t446 = 0.930418*t160*t428
  t447 = -0.366501*t160*t433
  t448 = t445 + t446 + t447
  t5 = 0.0016*t3*t4
  t13 = 0.2595*t12
  t17 = 0.0002*t16
  t466 = cos(var1[19+1])
  t467 = -1.0 * t466
  t468 = 1.0 +  t467
  t470 = sin(var1[19+1])
  t474 = sin(var1[18+1])
  t476 = cos(var1[18+1])
  t484 = -1.0 * t474*t12
  t485 = t476*t16
  t486 = t484 + t485
  t492 = t476*t12
  t493 = t474*t16
  t494 = t492 + t493
  t497 = cos(var1[20+1])
  t498 = -1.0 * t497
  t499 = 1.0 +  t498
  t501 = sin(var1[20+1])
  t504 = -0.366501*t3*t4*t470
  t505 = 0.340999127418*t468*t486
  t506 = -0.134322983001*t468
  t507 = 1.0 +  t506
  t508 = t507*t494
  t509 = t504 + t505 + t508
  t515 = 0.930418*t3*t4*t470
  t516 = -0.8656776547239999*t468
  t517 = 1.0 +  t516
  t518 = t517*t486
  t519 = 0.340999127418*t468*t494
  t520 = t515 + t518 + t519
  t525 = -1.000000637725*t468
  t526 = 1.0 +  t525
  t527 = t526*t3*t4
  t528 = -0.930418*t470*t486
  t529 = 0.366501*t470*t494
  t530 = t527 + t528 + t529
  t532 = cos(var1[21+1])
  t533 = -1.0 * t532
  t534 = 1.0 +  t533
  t536 = sin(var1[21+1])
  t539 = 0.930418*t501*t509
  t540 = 0.366501*t501*t520
  t541 = -1.000000637725*t499
  t542 = 1.0 +  t541
  t543 = t542*t530
  t544 = t539 + t540 + t543
  t550 = -0.8656776547239999*t499
  t551 = 1.0 +  t550
  t552 = t551*t509
  t553 = -0.340999127418*t499*t520
  t554 = -0.930418*t501*t530
  t555 = t552 + t553 + t554
  t561 = -0.340999127418*t499*t509
  t562 = -0.134322983001*t499
  t563 = 1.0 +  t562
  t564 = t563*t520
  t565 = -0.366501*t501*t530
  t566 = t561 + t564 + t565
  t569 = cos(var1[22+1])
  t570 = -1.0 * t569
  t571 = 1.0 +  t570
  t573 = sin(var1[22+1])
  t576 = 0.366501*t536*t544
  t577 = -0.340999127418*t534*t555
  t578 = -0.134322983001*t534
  t579 = 1.0 +  t578
  t580 = t579*t566
  t581 = t576 + t577 + t580
  t587 = 0.930418*t536*t544
  t588 = -0.8656776547239999*t534
  t589 = 1.0 +  t588
  t590 = t589*t555
  t591 = -0.340999127418*t534*t566
  t592 = t587 + t590 + t591
  t597 = -1.000000637725*t534
  t598 = 1.0 +  t597
  t599 = t598*t544
  t600 = -0.930418*t536*t555
  t601 = -0.366501*t536*t566
  t602 = t599 + t600 + t601
  t604 = cos(var1[23+1])
  t605 = -1.0 * t604
  t606 = 1.0 +  t605
  t608 = sin(var1[23+1])
  t611 = -0.366501*t573*t581
  t612 = -0.930418*t573*t592
  t613 = -1.000000637725*t571
  t614 = 1.0 +  t613
  t615 = t614*t602
  t616 = t611 + t612 + t615
  t622 = -0.134322983001*t571
  t623 = 1.0 +  t622
  t624 = t623*t581
  t625 = -0.340999127418*t571*t592
  t626 = 0.366501*t573*t602
  t627 = t624 + t625 + t626
  t633 = -0.340999127418*t571*t581
  t634 = -0.8656776547239999*t571
  t635 = 1.0 +  t634
  t636 = t635*t592
  t637 = 0.930418*t573*t602
  t638 = t633 + t636 + t637
  t641 = cos(var1[24+1])
  t642 = -1.0 * t641
  t643 = 1.0 +  t642
  t645 = sin(var1[24+1])
  t648 = 0.930418*t608*t616
  t649 = -0.340999127418*t606*t627
  t650 = -0.8656776547239999*t606
  t651 = 1.0 +  t650
  t652 = t651*t638
  t653 = t648 + t649 + t652
  t659 = 0.366501*t608*t616
  t660 = -0.134322983001*t606
  t661 = 1.0 +  t660
  t662 = t661*t627
  t663 = -0.340999127418*t606*t638
  t664 = t659 + t662 + t663
  t669 = -1.000000637725*t606
  t670 = 1.0 +  t669
  t671 = t670*t616
  t672 = -0.366501*t608*t627
  t673 = -0.930418*t608*t638
  t674 = t671 + t672 + t673
  t250 = 0.0016*t4*t9
  t469 = -0.04500040093286238*t468
  t471 = 0.0846680539949003*t470
  t472 = t469 + t471
  t254 = 0.2595*t253
  t258 = 0.0002*t257
  t477 = -1.0 * t476
  t478 = 1.0 +  t477
  t480 = 1.296332362046933e-7*var1[19+1]
  t481 = -0.07877668146182712*t468
  t482 = -0.04186915633414423*t470
  t483 = t480 + t481 + t482
  t488 = 3.2909349868922137e-7*var1[19+1]
  t489 = 0.03103092645718495*t468
  t490 = 0.016492681424499736*t470
  t491 = t488 + t489 + t490
  t496 = -1.296332362046933e-7*var1[20+1]
  t500 = -0.14128592423750855*t499
  t502 = 0.04186915633414423*t501
  t503 = t496 + t500 + t502
  t701 = -1.0 * t474*t253
  t702 = t476*t257
  t703 = t701 + t702
  t705 = t476*t253
  t706 = t474*t257
  t707 = t705 + t706
  t511 = 3.2909349868922137e-7*var1[20+1]
  t512 = -0.055653945343889656*t499
  t513 = 0.016492681424499736*t501
  t514 = t511 + t512 + t513
  t522 = -0.04500040093286238*t499
  t523 = -0.15185209683981668*t501
  t524 = t522 + t523
  t535 = 0.039853038461262744*t534
  t537 = 0.23670515095269612*t536
  t538 = t535 + t537
  t709 = -0.366501*t4*t470*t9
  t710 = 0.340999127418*t468*t703
  t711 = t507*t707
  t712 = t709 + t710 + t711
  t714 = 0.930418*t4*t470*t9
  t715 = t517*t703
  t716 = 0.340999127418*t468*t707
  t717 = t714 + t715 + t716
  t719 = t526*t4*t9
  t720 = -0.930418*t470*t703
  t721 = 0.366501*t470*t707
  t722 = t719 + t720 + t721
  t546 = 6.295460977284962e-8*var1[21+1]
  t547 = -0.22023473313910558*t534
  t548 = 0.03707996069223323*t536
  t549 = t546 + t547 + t548
  t557 = -1.5981976069815686e-7*var1[21+1]
  t558 = -0.08675267452931407*t534
  t559 = 0.014606169134372047*t536
  t560 = t557 + t558 + t559
  t568 = -4.0833068682577724e-7*var1[22+1]
  t572 = -0.11476729583292707*t571
  t574 = 0.0111594154470601*t573
  t575 = t568 + t572 + t574
  t724 = 0.930418*t501*t712
  t725 = 0.366501*t501*t717
  t726 = t542*t722
  t727 = t724 + t725 + t726
  t729 = t551*t712
  t730 = -0.340999127418*t499*t717
  t731 = -0.930418*t501*t722
  t732 = t729 + t730 + t731
  t734 = -0.340999127418*t499*t712
  t735 = t563*t717
  t736 = -0.366501*t501*t722
  t737 = t734 + t735 + t736
  t583 = 1.6084556086870008e-7*var1[22+1]
  t584 = -0.29135406957765553*t571
  t585 = 0.02832985722118838*t573
  t586 = t583 + t584 + t585
  t594 = 0.03044854601678662*t571
  t595 = 0.3131431996991197*t573
  t596 = t594 + t595
  t607 = -0.26285954081199375*t606
  t609 = 0.634735404786378*t608
  t610 = t607 + t609
  t739 = 0.366501*t536*t727
  t740 = -0.340999127418*t534*t732
  t741 = t579*t737
  t742 = t739 + t740 + t741
  t744 = 0.930418*t536*t727
  t745 = t589*t732
  t746 = -0.340999127418*t534*t737
  t747 = t744 + t745 + t746
  t749 = t598*t727
  t750 = -0.930418*t536*t732
  t751 = -0.366501*t536*t737
  t752 = t749 + t750 + t751
  t618 = 1.6169269214444473e-7*var1[23+1]
  t619 = -0.2326311605896123*t606
  t620 = -0.09633822312984319*t608
  t621 = t618 + t619 + t620
  t629 = -6.369237629068993e-8*var1[23+1]
  t630 = -0.5905692458505322*t606
  t631 = -0.24456909227538925*t608
  t632 = t629 + t630 + t631
  t640 = -7.041766963257243e-8*var1[24+1]
  t644 = -0.8232948486053725*t643
  t646 = 0.05763710717422546*t645
  t647 = t640 + t644 + t646
  t754 = -0.366501*t573*t742
  t755 = -0.930418*t573*t747
  t756 = t614*t752
  t757 = t754 + t755 + t756
  t759 = t623*t742
  t760 = -0.340999127418*t571*t747
  t761 = 0.366501*t573*t752
  t762 = t759 + t760 + t761
  t764 = -0.340999127418*t571*t742
  t765 = t635*t747
  t766 = 0.930418*t573*t752
  t767 = t764 + t765 + t766
  t655 = 1.7876586242383724e-7*var1[24+1]
  t656 = -0.3243041141817093*t643
  t657 = 0.02270383571304597*t645
  t658 = t655 + t656 + t657
  t666 = 0.06194758047549556*t643
  t667 = 0.8848655643005321*t645
  t668 = t666 + t667
  t769 = 0.930418*t608*t757
  t770 = -0.340999127418*t606*t762
  t771 = t651*t767
  t772 = t769 + t770 + t771
  t774 = 0.366501*t608*t757
  t775 = t661*t762
  t776 = -0.340999127418*t606*t767
  t777 = t774 + t775 + t776
  t678 = -1.000000637725*t643
  t679 = 1.0 +  t678
  t779 = t670*t757
  t780 = -0.366501*t608*t762
  t781 = -0.930418*t608*t767
  t782 = t779 + t780 + t781
  t684 = -0.134322983001*t643
  t685 = 1.0 +  t684
  t690 = -0.8656776547239999*t643
  t691 = 1.0 +  t690
  t361 = 0.2595*t4*t6
  t362 = -0.0016*t7
  t363 = 0.0002*t4*t10
  t803 = -1.0 * t4*t6*t474
  t804 = t476*t4*t10
  t805 = t803 + t804
  t807 = t476*t4*t6
  t808 = t4*t474*t10
  t809 = t807 + t808
  t811 = 0.366501*t470*t7
  t812 = 0.340999127418*t468*t805
  t813 = t507*t809
  t814 = t811 + t812 + t813
  t816 = -0.930418*t470*t7
  t817 = t517*t805
  t818 = 0.340999127418*t468*t809
  t819 = t816 + t817 + t818
  t821 = -1.0 * t526*t7
  t822 = -0.930418*t470*t805
  t823 = 0.366501*t470*t809
  t824 = t821 + t822 + t823
  t826 = 0.930418*t501*t814
  t827 = 0.366501*t501*t819
  t828 = t542*t824
  t829 = t826 + t827 + t828
  t831 = t551*t814
  t832 = -0.340999127418*t499*t819
  t833 = -0.930418*t501*t824
  t834 = t831 + t832 + t833
  t836 = -0.340999127418*t499*t814
  t837 = t563*t819
  t838 = -0.366501*t501*t824
  t839 = t836 + t837 + t838
  t841 = 0.366501*t536*t829
  t842 = -0.340999127418*t534*t834
  t843 = t579*t839
  t844 = t841 + t842 + t843
  t846 = 0.930418*t536*t829
  t847 = t589*t834
  t848 = -0.340999127418*t534*t839
  t849 = t846 + t847 + t848
  t851 = t598*t829
  t852 = -0.930418*t536*t834
  t853 = -0.366501*t536*t839
  t854 = t851 + t852 + t853
  t856 = -0.366501*t573*t844
  t857 = -0.930418*t573*t849
  t858 = t614*t854
  t859 = t856 + t857 + t858
  t861 = t623*t844
  t862 = -0.340999127418*t571*t849
  t863 = 0.366501*t573*t854
  t864 = t861 + t862 + t863
  t866 = -0.340999127418*t571*t844
  t867 = t635*t849
  t868 = 0.930418*t573*t854
  t869 = t866 + t867 + t868
  t871 = 0.930418*t608*t859
  t872 = -0.340999127418*t606*t864
  t873 = t651*t869
  t874 = t871 + t872 + t873
  t876 = 0.366501*t608*t859
  t877 = t661*t864
  t878 = -0.340999127418*t606*t869
  t879 = t876 + t877 + t878
  t881 = t670*t859
  t882 = -0.366501*t608*t864
  t883 = -0.930418*t608*t869
  t884 = t881 + t882 + t883
  p_output1[0+1]=-1.0 * t101*t107 - 1.0 * t112*t118 + t13 - 1.0 * t127*t133 - 1.0 * t138*t144 - 1.0 * t148*t154 - 1.0 * t162*t168 + t17 - 1.0 * t173*t179 - 1.0 * t184*t190 - 0.091*t16*t20 - 1.0 * t199*t205 - 1.0 * t210*t216 - 0.091*t12*t22 - 1.0 * t220*t226 - 0.061947*(-0.366501*t197*t205 + 0.930418*t197*t216 + t226*t231) + 0.792446*(0.340999127418*t195*t205 - 0.930418*t197*t226 + t216*t237) - 0.402615*(0.340999127418*t195*t216 + 0.366501*t197*t226 + t205*t243) - 1.0 * t34*t38 - 1.0 * t3*t30*t4 - 1.0 * t42*t46 + t5 - 1.0 * t53*t61 - 1.0 * t68*t72 - 1.0 * t79*t82 - 1.0 * t90*t96
  p_output1[1+1]=t250 - 0.091*t22*t253 + t254 - 0.091*t20*t257 + t258 - 1.0 * t101*t293 - 1.0 * t112*t298 - 1.0 * t127*t303 - 1.0 * t138*t308 - 1.0 * t148*t313 - 1.0 * t162*t318 - 1.0 * t173*t323 - 1.0 * t184*t328 - 1.0 * t199*t333 - 1.0 * t210*t338 - 1.0 * t220*t343 + 0.792446*(0.340999127418*t195*t333 + t237*t338 - 0.930418*t197*t343) - 0.402615*(t243*t333 + 0.340999127418*t195*t338 + 0.366501*t197*t343) - 0.061947*(-0.366501*t197*t333 + 0.930418*t197*t338 + t231*t343) - 1.0 * t264*t38 - 1.0 * t268*t46 - 1.0 * t273*t61 - 1.0 * t278*t72 - 1.0 * t283*t82 - 1.0 * t30*t4*t9 - 1.0 * t288*t90
  p_output1[2+1]=t361 + t362 + t363 - 1.0 * t369*t38 - 1.0 * t101*t398 - 0.091*t10*t20*t4 - 1.0 * t112*t403 - 1.0 * t127*t408 - 1.0 * t138*t413 - 1.0 * t148*t418 - 1.0 * t162*t423 - 1.0 * t173*t428 - 1.0 * t184*t433 - 1.0 * t199*t438 - 1.0 * t210*t443 - 1.0 * t220*t448 + 0.792446*(0.340999127418*t195*t438 + t237*t443 - 0.930418*t197*t448) - 0.402615*(t243*t438 + 0.340999127418*t195*t443 + 0.366501*t197*t448) - 0.061947*(-0.366501*t197*t438 + 0.930418*t197*t443 + t231*t448) - 1.0 * t373*t46 - 0.091*t22*t4*t6 - 1.0 * t378*t61 + t30*t7 - 1.0 * t383*t72 - 1.0 * t388*t82 - 1.0 * t393*t90
  p_output1[3+1]=t13 + t17 - 1.0 * t3*t4*t472 + 0.091*t12*t474 + 0.091*t16*t478 - 1.0 * t483*t486 - 1.0 * t491*t494 + t5 - 1.0 * t503*t509 - 1.0 * t514*t520 - 1.0 * t524*t530 - 1.0 * t538*t544 - 1.0 * t549*t555 - 1.0 * t560*t566 - 1.0 * t575*t581 - 1.0 * t586*t592 - 1.0 * t596*t602 - 1.0 * t610*t616 - 1.0 * t621*t627 - 1.0 * t632*t638 - 1.0 * t647*t653 - 1.0 * t658*t664 - 1.0 * t668*t674 - 0.061947*(-0.930418*t645*t653 - 0.366501*t645*t664 + t674*t679) + 0.402615*(-0.340999127418*t643*t653 + 0.366501*t645*t674 + t664*t685) + 0.792446*(-0.340999127418*t643*t664 + 0.930418*t645*t674 + t653*t691)
  p_output1[4+1]=t250 + t254 + t258 + 0.091*t253*t474 + 0.091*t257*t478 - 1.0 * t483*t703 - 1.0 * t491*t707 - 1.0 * t503*t712 - 1.0 * t514*t717 - 1.0 * t524*t722 - 1.0 * t538*t727 - 1.0 * t549*t732 - 1.0 * t560*t737 - 1.0 * t575*t742 - 1.0 * t586*t747 - 1.0 * t596*t752 - 1.0 * t610*t757 - 1.0 * t621*t762 - 1.0 * t632*t767 - 1.0 * t647*t772 - 1.0 * t658*t777 - 1.0 * t668*t782 + 0.402615*(-0.340999127418*t643*t772 + t685*t777 + 0.366501*t645*t782) + 0.792446*(t691*t772 - 0.340999127418*t643*t777 + 0.930418*t645*t782) - 0.061947*(-0.930418*t645*t772 - 0.366501*t645*t777 + t679*t782) - 1.0 * t4*t472*t9
  p_output1[5+1]=t361 + t362 + t363 + 0.091*t10*t4*t478 + 0.091*t4*t474*t6 + t472*t7 - 1.0 * t483*t805 - 1.0 * t491*t809 - 1.0 * t503*t814 - 1.0 * t514*t819 - 1.0 * t524*t824 - 1.0 * t538*t829 - 1.0 * t549*t834 - 1.0 * t560*t839 - 1.0 * t575*t844 - 1.0 * t586*t849 - 1.0 * t596*t854 - 1.0 * t610*t859 - 1.0 * t621*t864 - 1.0 * t632*t869 - 1.0 * t647*t874 - 1.0 * t658*t879 - 1.0 * t668*t884 + 0.402615*(-0.340999127418*t643*t874 + t685*t879 + 0.366501*t645*t884) + 0.792446*(t691*t874 - 0.340999127418*t643*t879 + 0.930418*t645*t884) - 0.061947*(-0.930418*t645*t874 - 0.366501*t645*t879 + t679*t884)
end



function p_base_wrt_feet(var1)
	p_output1 = zeros(6)
	base_wrt_feet_helper!(p_output1, var1)
	return p_output1
end 