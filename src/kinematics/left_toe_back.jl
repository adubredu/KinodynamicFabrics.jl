function left_toe_back_helper!(p_output1, var1)
  t471 = cos(var1[3+1])
  t382 = cos(var1[5+1])
  t485 = sin(var1[4+1])
  t414 = sin(var1[3+1])
  t489 = sin(var1[5+1])
  t338 = cos(var1[6+1])
  t429 = -1.0 * t382*t414
  t508 = t471*t485*t489
  t548 = t429 + t508
  t565 = t471*t382*t485
  t566 = t414*t489
  t567 = t565 + t566
  t568 = sin(var1[6+1])
  t609 = cos(var1[7+1])
  t610 = -1.0 * t609
  t611 = 1.0 +  t610
  t628 = sin(var1[7+1])
  t663 = t338*t548
  t668 = -1.0 * t567*t568
  t676 = t663 + t668
  t716 = t338*t567
  t728 = t548*t568
  t729 = t716 + t728
  t608 = cos(var1[4+1])
  t785 = cos(var1[8+1])
  t786 = -1.0 * t785
  t787 = 1.0 +  t786
  t803 = sin(var1[8+1])
  t862 = -1.000000637725*t611
  t866 = 1.0 +  t862
  t868 = t471*t608*t866
  t869 = -0.930418*t676*t628
  t872 = -0.366501*t729*t628
  t874 = t868 + t869 + t872
  t750 = -0.340999127418*t611*t676
  t752 = -0.134322983001*t611
  t760 = 1.0 +  t752
  t761 = t760*t729
  t765 = 0.366501*t471*t608*t628
  t766 = t750 + t761 + t765
  t818 = -0.8656776547239999*t611
  t827 = 1.0 +  t818
  t830 = t827*t676
  t831 = -0.340999127418*t611*t729
  t834 = 0.930418*t471*t608*t628
  t835 = t830 + t831 + t834
  t894 = cos(var1[9+1])
  t895 = -1.0 * t894
  t896 = 1.0 +  t895
  t903 = sin(var1[9+1])
  t909 = -1.000000637725*t787
  t910 = 1.0 +  t909
  t916 = t910*t874
  t919 = -0.930418*t766*t803
  t920 = 0.366501*t835*t803
  t921 = t916 + t919 + t920
  t939 = 0.340999127418*t787*t766
  t941 = -0.134322983001*t787
  t942 = 1.0 +  t941
  t945 = t942*t835
  t946 = -0.366501*t874*t803
  t947 = t939 + t945 + t946
  t972 = -0.8656776547239999*t787
  t973 = 1.0 +  t972
  t976 = t973*t766
  t977 = 0.340999127418*t787*t835
  t978 = 0.930418*t874*t803
  t979 = t976 + t977 + t978
  t984 = cos(var1[10+1])
  t985 = -1.0 * t984
  t988 = 1.0 +  t985
  t990 = sin(var1[10+1])
  t997 = -0.930418*t903*t921
  t1000 = 0.340999127418*t896*t947
  t1001 = -0.8656776547239999*t896
  t1002 = 1.0 +  t1001
  t1003 = t1002*t979
  t1006 = t997 + t1000 + t1003
  t1018 = 0.366501*t903*t921
  t1020 = -0.134322983001*t896
  t1025 = 1.0 +  t1020
  t1026 = t1025*t947
  t1028 = 0.340999127418*t896*t979
  t1030 = t1018 + t1026 + t1028
  t1042 = -1.000000637725*t896
  t1044 = 1.0 +  t1042
  t1047 = t1044*t921
  t1048 = -0.366501*t903*t947
  t1049 = 0.930418*t903*t979
  t1051 = t1047 + t1048 + t1049
  t1058 = cos(var1[11+1])
  t1059 = -1.0 * t1058
  t1061 = 1.0 +  t1059
  t1063 = sin(var1[11+1])
  t1070 = 0.930418*t990*t1006
  t1073 = -0.366501*t990*t1030
  t1074 = -1.000000637725*t988
  t1075 = 1.0 +  t1074
  t1077 = t1075*t1051
  t1084 = t1070 + t1073 + t1077
  t1094 = -0.8656776547239999*t988
  t1098 = 1.0 +  t1094
  t1099 = t1098*t1006
  t1100 = 0.340999127418*t988*t1030
  t1102 = -0.930418*t990*t1051
  t1103 = t1099 + t1100 + t1102
  t1118 = 0.340999127418*t988*t1006
  t1119 = -0.134322983001*t988
  t1120 = 1.0 +  t1119
  t1123 = t1120*t1030
  t1124 = 0.366501*t990*t1051
  t1125 = t1118 + t1123 + t1124
  t1130 = cos(var1[12+1])
  t1137 = -1.0 * t1130
  t1143 = 1.0 +  t1137
  t1147 = sin(var1[12+1])
  t1217 = cos(var1[13+1])
  t1221 = -1.0 * t1217
  t1223 = 1.0 +  t1221
  t1228 = sin(var1[13+1])
  t1154 = 0.366501*t1063*t1084
  t1155 = 0.340999127418*t1061*t1103
  t1157 = -0.134322983001*t1061
  t1162 = 1.0 +  t1157
  t1163 = t1162*t1125
  t1164 = t1154 + t1155 + t1163
  t1177 = -0.930418*t1063*t1084
  t1178 = -0.8656776547239999*t1061
  t1179 = 1.0 +  t1178
  t1182 = t1179*t1103
  t1184 = 0.340999127418*t1061*t1125
  t1185 = t1177 + t1182 + t1184
  t1199 = -1.000000637725*t1061
  t1200 = 1.0 +  t1199
  t1201 = t1200*t1084
  t1202 = 0.930418*t1063*t1103
  t1211 = -0.366501*t1063*t1125
  t1213 = t1201 + t1202 + t1211
  t1225 = -0.444895486988*t1223
  t1235 = 0.175248972904*t1223
  t1261 = 0.120666640478*t1223
  t1281 = -0.553471*t1228
  t1246 = -0.366501*t1147*t1164
  t1247 = 0.930418*t1147*t1185
  t1251 = -1.000000637725*t1143
  t1254 = 1.0 +  t1251
  t1255 = t1254*t1213
  t1256 = t1246 + t1247 + t1255
  t1284 = 0.803828*t1228
  t1271 = 0.340999127418*t1143*t1164
  t1272 = -0.8656776547239999*t1143
  t1273 = 1.0 +  t1272
  t1274 = t1273*t1185
  t1276 = -0.930418*t1147*t1213
  t1277 = t1271 + t1274 + t1276
  t1289 = -0.134322983001*t1143
  t1290 = 1.0 +  t1289
  t1291 = t1290*t1164
  t1292 = 0.340999127418*t1143*t1185
  t1295 = 0.366501*t1147*t1213
  t1296 = t1291 + t1292 + t1295
  t1267 = -0.218018*t1228
  t1303 = -0.120666640478*t1223
  t1263 = -0.803828*t1228
  t1314 = 0.444895486988*t1223
  t1230 = 0.218018*t1228
  t1300 = -0.175248972904*t1223
  t1237 = 0.553471*t1228
  t366 = -1.0 * t338
  t376 = 1.0 +  t366
  t620 = -0.04500040093286238*t611
  t629 = -0.0846680539949003*t628
  t646 = t620 + t629
  t1337 = t471*t382
  t1338 = t414*t485*t489
  t1339 = t1337 + t1338
  t1341 = t382*t414*t485
  t1342 = -1.0 * t471*t489
  t1343 = t1341 + t1342
  t677 = 1.296332362046933e-7*var1[7+1]
  t703 = 0.07877668146182712*t611
  t708 = -0.04186915633414423*t628
  t709 = t677 + t703 + t708
  t730 = -3.2909349868922137e-7*var1[7+1]
  t739 = 0.03103092645718495*t611
  t745 = -0.016492681424499736*t628
  t746 = t730 + t739 + t745
  t1346 = t338*t1339
  t1347 = -1.0 * t1343*t568
  t1348 = t1346 + t1347
  t1350 = t338*t1343
  t1351 = t1339*t568
  t1352 = t1350 + t1351
  t784 = 1.296332362046933e-7*var1[8+1]
  t793 = -0.14128592423750855*t787
  t807 = -0.04186915633414423*t803
  t808 = t784 + t793 + t807
  t838 = 3.2909349868922137e-7*var1[8+1]
  t851 = 0.055653945343889656*t787
  t852 = 0.016492681424499736*t803
  t857 = t838 + t851 + t852
  t879 = -0.04500040093286238*t787
  t883 = 0.15185209683981668*t803
  t890 = t879 + t883
  t902 = 0.039853038461262744*t896
  t904 = -0.23670515095269612*t903
  t907 = t902 + t904
  t1364 = t608*t866*t414
  t1365 = -0.930418*t1348*t628
  t1366 = -0.366501*t1352*t628
  t1367 = t1364 + t1365 + t1366
  t1354 = -0.340999127418*t611*t1348
  t1355 = t760*t1352
  t1356 = 0.366501*t608*t414*t628
  t1357 = t1354 + t1355 + t1356
  t1359 = t827*t1348
  t1360 = -0.340999127418*t611*t1352
  t1361 = 0.930418*t608*t414*t628
  t1362 = t1359 + t1360 + t1361
  t926 = -1.5981976069815686e-7*var1[9+1]
  t930 = 0.08675267452931407*t896
  t931 = 0.014606169134372047*t903
  t936 = t926 + t930 + t931
  t954 = -6.295460977284962e-8*var1[9+1]
  t955 = -0.22023473313910558*t896
  t958 = -0.03707996069223323*t903
  t959 = t954 + t955 + t958
  t983 = -1.6084556086870008e-7*var1[10+1]
  t989 = -0.29135406957765553*t988
  t995 = -0.02832985722118838*t990
  t996 = t983 + t989 + t995
  t1369 = t910*t1367
  t1370 = -0.930418*t1357*t803
  t1371 = 0.366501*t1362*t803
  t1373 = t1369 + t1370 + t1371
  t1375 = 0.340999127418*t787*t1357
  t1376 = t942*t1362
  t1378 = -0.366501*t1367*t803
  t1379 = t1375 + t1376 + t1378
  t1381 = t973*t1357
  t1383 = 0.340999127418*t787*t1362
  t1384 = 0.930418*t1367*t803
  t1385 = t1381 + t1383 + t1384
  t1009 = -4.0833068682577724e-7*var1[10+1]
  t1012 = 0.11476729583292707*t988
  t1016 = 0.0111594154470601*t990
  t1017 = t1009 + t1012 + t1016
  t1032 = 0.03044854601678662*t988
  t1038 = -0.3131431996991197*t990
  t1041 = t1032 + t1038
  t1062 = -0.26285954081199375*t1061
  t1064 = -0.634735404786378*t1063
  t1069 = t1062 + t1064
  t1387 = -0.930418*t903*t1373
  t1389 = 0.340999127418*t896*t1379
  t1391 = t1002*t1385
  t1393 = t1387 + t1389 + t1391
  t1395 = 0.366501*t903*t1373
  t1397 = t1025*t1379
  t1398 = 0.340999127418*t896*t1385
  t1399 = t1395 + t1397 + t1398
  t1403 = t1044*t1373
  t1404 = -0.366501*t903*t1379
  t1406 = 0.930418*t903*t1385
  t1407 = t1403 + t1404 + t1406
  t1087 = 6.369237629068993e-8*var1[11+1]
  t1090 = -0.5905692458505322*t1061
  t1092 = 0.24456909227538925*t1063
  t1093 = t1087 + t1090 + t1092
  t1112 = 1.6169269214444473e-7*var1[11+1]
  t1113 = 0.2326311605896123*t1061
  t1114 = -0.09633822312984319*t1063
  t1117 = t1112 + t1113 + t1114
  t1129 = 1.7876586242383724e-7*var1[12+1]
  t1145 = 0.3243041141817093*t1143
  t1150 = 0.02270383571304597*t1147
  t1152 = t1129 + t1145 + t1150
  t1409 = 0.930418*t990*t1393
  t1411 = -0.366501*t990*t1399
  t1414 = t1075*t1407
  t1415 = t1409 + t1411 + t1414
  t1419 = t1098*t1393
  t1420 = 0.340999127418*t988*t1399
  t1421 = -0.930418*t990*t1407
  t1424 = t1419 + t1420 + t1421
  t1426 = 0.340999127418*t988*t1393
  t1427 = t1120*t1399
  t1429 = 0.366501*t990*t1407
  t1430 = t1426 + t1427 + t1429
  t1169 = 7.041766963257243e-8*var1[12+1]
  t1173 = -0.8232948486053725*t1143
  t1174 = -0.05763710717422546*t1147
  t1175 = t1169 + t1173 + t1174
  t1189 = 0.06194758047549556*t1143
  t1196 = -0.8848655643005321*t1147
  t1197 = t1189 + t1196
  t1215 = -2.7989049814696287e-7*var1[13+1]
  t1224 = 0.15748067958019524*t1223
  t1231 = t1225 + t1230
  t1234 = -0.528674719304*t1231
  t1240 = t1235 + t1237
  t1241 = 0.29871295412*t1240
  t1242 = t1215 + t1224 + t1234 + t1241
  t1433 = 0.366501*t1063*t1415
  t1435 = 0.340999127418*t1061*t1424
  t1436 = t1162*t1430
  t1438 = t1433 + t1435 + t1436
  t1441 = -0.930418*t1063*t1415
  t1443 = t1179*t1424
  t1444 = 0.340999127418*t1061*t1430
  t1445 = t1441 + t1443 + t1444
  t1448 = t1200*t1415
  t1449 = 0.930418*t1063*t1424
  t1450 = -0.366501*t1063*t1430
  t1453 = t1448 + t1449 + t1450
  t1259 = -1.9271694180831932e-7*var1[13+1]
  t1260 = -0.3667264808254521*t1223
  t1265 = t1261 + t1263
  t1266 = 0.29871295412*t1265
  t1268 = t1225 + t1267
  t1269 = 0.445034169498*t1268
  t1270 = t1259 + t1260 + t1266 + t1269
  t1279 = 7.591321355439789e-8*var1[13+1]
  t1280 = 0.2845150083511607*t1223
  t1282 = t1235 + t1281
  t1283 = 0.445034169498*t1282
  t1286 = t1261 + t1284
  t1287 = -0.528674719304*t1286
  t1288 = t1279 + t1280 + t1283 + t1287
  t1301 = t1300 + t1281
  t1455 = -0.366501*t1147*t1438
  t1456 = 0.930418*t1147*t1445
  t1459 = t1254*t1453
  t1460 = t1455 + t1456 + t1459
  t1304 = t1303 + t1284
  t1463 = 0.340999127418*t1143*t1438
  t1465 = t1273*t1445
  t1466 = -0.930418*t1147*t1453
  t1467 = t1463 + t1465 + t1466
  t1306 = -0.952469601425*t1223
  t1307 = 1.0 +  t1306
  t1470 = t1290*t1438
  t1471 = 0.340999127418*t1143*t1445
  t1472 = 0.366501*t1147*t1453
  t1473 = t1470 + t1471 + t1472
  t1315 = t1314 + t1267
  t1318 = -0.693671301908*t1223
  t1319 = 1.0 +  t1318
  t1321 = t1303 + t1263
  t1326 = -0.353861996165*t1223
  t1327 = 1.0 +  t1326
  t1330 = t1314 + t1230
  t1332 = t1300 + t1237
  t1500 = t608*t338*t489
  t1501 = -1.0 * t608*t382*t568
  t1502 = t1500 + t1501
  t1505 = t608*t382*t338
  t1506 = t608*t489*t568
  t1507 = t1505 + t1506
  t1519 = -1.0 * t866*t485
  t1520 = -0.930418*t1502*t628
  t1521 = -0.366501*t1507*t628
  t1522 = t1519 + t1520 + t1521
  t1514 = t827*t1502
  t1515 = -0.340999127418*t611*t1507
  t1516 = -0.930418*t485*t628
  t1517 = t1514 + t1515 + t1516
  t1509 = -0.340999127418*t611*t1502
  t1510 = t760*t1507
  t1511 = -0.366501*t485*t628
  t1512 = t1509 + t1510 + t1511
  t1524 = t910*t1522
  t1525 = 0.366501*t1517*t803
  t1526 = -0.930418*t1512*t803
  t1527 = t1524 + t1525 + t1526
  t1530 = t942*t1517
  t1531 = 0.340999127418*t787*t1512
  t1532 = -0.366501*t1522*t803
  t1533 = t1530 + t1531 + t1532
  t1535 = 0.340999127418*t787*t1517
  t1536 = t973*t1512
  t1537 = 0.930418*t1522*t803
  t1538 = t1535 + t1536 + t1537
  t1540 = -0.930418*t903*t1527
  t1541 = 0.340999127418*t896*t1533
  t1543 = t1002*t1538
  t1545 = t1540 + t1541 + t1543
  t1547 = 0.366501*t903*t1527
  t1549 = t1025*t1533
  t1550 = 0.340999127418*t896*t1538
  t1552 = t1547 + t1549 + t1550
  t1554 = t1044*t1527
  t1558 = -0.366501*t903*t1533
  t1559 = 0.930418*t903*t1538
  t1560 = t1554 + t1558 + t1559
  t1563 = 0.930418*t990*t1545
  t1564 = -0.366501*t990*t1552
  t1565 = t1075*t1560
  t1567 = t1563 + t1564 + t1565
  t1569 = t1098*t1545
  t1570 = 0.340999127418*t988*t1552
  t1572 = -0.930418*t990*t1560
  t1574 = t1569 + t1570 + t1572
  t1577 = 0.340999127418*t988*t1545
  t1578 = t1120*t1552
  t1579 = 0.366501*t990*t1560
  t1580 = t1577 + t1578 + t1579
  t1583 = 0.366501*t1063*t1567
  t1584 = 0.340999127418*t1061*t1574
  t1585 = t1162*t1580
  t1587 = t1583 + t1584 + t1585
  t1589 = -0.930418*t1063*t1567
  t1590 = t1179*t1574
  t1592 = 0.340999127418*t1061*t1580
  t1593 = t1589 + t1590 + t1592
  t1595 = t1200*t1567
  t1597 = 0.930418*t1063*t1574
  t1598 = -0.366501*t1063*t1580
  t1599 = t1595 + t1597 + t1598
  t1602 = -0.366501*t1147*t1587
  t1604 = 0.930418*t1147*t1593
  t1605 = t1254*t1599
  t1607 = t1602 + t1604 + t1605
  t1610 = 0.340999127418*t1143*t1587
  t1612 = t1273*t1593
  t1613 = -0.930418*t1147*t1599
  t1614 = t1610 + t1612 + t1613
  t1618 = t1290*t1587
  t1619 = 0.340999127418*t1143*t1593
  t1620 = 0.366501*t1147*t1599
  t1622 = t1618 + t1619 + t1620
  p_output1[0+1]=t1017*t1030 + t1041*t1051 + t1069*t1084 + t1093*t1103 + t1117*t1125 + t1152*t1164 + t1175*t1185 + t1197*t1213 + t1242*t1256 + t1270*t1277 + t1288*t1296 + 0.430001*(t1256*t1301 + t1277*t1304 + t1296*t1307) - 0.861971*(t1256*t1315 + t1277*t1319 + t1296*t1321) - 0.037329*(t1256*t1327 + t1277*t1330 + t1296*t1332) + 0.091*t376*t548 + 0.091*t567*t568 + t471*t608*t646 + t676*t709 + t729*t746 + t766*t808 + t835*t857 + t874*t890 + t907*t921 + t936*t947 + t959*t979 + t1006*t996 + var1[0+1]
  p_output1[1+1]=t1017*t1399 + t1041*t1407 + t1069*t1415 + t1093*t1424 + t1117*t1430 + t1152*t1438 + t1175*t1445 + t1197*t1453 + t1242*t1460 + t1270*t1467 + t1288*t1473 + 0.430001*(t1301*t1460 + t1304*t1467 + t1307*t1473) - 0.861971*(t1315*t1460 + t1319*t1467 + t1321*t1473) - 0.037329*(t1327*t1460 + t1330*t1467 + t1332*t1473) + 0.091*t1339*t376 + 0.091*t1343*t568 + t414*t608*t646 + t1348*t709 + t1352*t746 + t1357*t808 + t1362*t857 + t1367*t890 + t1373*t907 + t1379*t936 + t1385*t959 + t1393*t996 + var1[1+1]
  p_output1[2+1]=t1017*t1552 + t1041*t1560 + t1069*t1567 + t1093*t1574 + t1117*t1580 + t1152*t1587 + t1175*t1593 + t1197*t1599 + t1242*t1607 + t1270*t1614 + t1288*t1622 + 0.430001*(t1301*t1607 + t1304*t1614 + t1307*t1622) - 0.861971*(t1315*t1607 + t1319*t1614 + t1321*t1622) - 0.037329*(t1327*t1607 + t1330*t1614 + t1332*t1622) + 0.091*t376*t489*t608 + 0.091*t382*t568*t608 - 1.0 * t485*t646 + t1502*t709 + t1507*t746 + t1512*t808 + t1517*t857 + t1522*t890 + t1527*t907 + t1533*t936 + t1538*t959 + t1545*t996 + var1[2+1]
end



function left_toe_back(var1)
	p_output1 = zeros(3)
	left_toe_back_helper!(p_output1, var1)
	return p_output1 
end
