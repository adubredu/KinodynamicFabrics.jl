function p_left_hand_wrt_base_helper!(p_output1, var1) 
  t2369 = cos(var1[3+1])
  t2483 = cos(var1[14+1])
  t2505 = -1.0*t2483
  t2506 = 1.0 + t2505
  t2517 = sin(var1[14+1])
  t2528 = cos(var1[5+1])
  t2531 = sin(var1[3+1])
  t2529 = sin(var1[4+1])
  t2532 = sin(var1[5+1])
  t2377 = cos(var1[15+1])
  t2407 = -1.0*t2377
  t2437 = 1.0 + t2407
  t2464 = sin(var1[15+1])
  t2530 = t2369*t2528*t2529
  t2533 = t2531*t2532
  t2538 = t2530 + t2533
  t2563 = -1.0*t2528*t2531
  t2565 = t2369*t2529*t2532
  t2566 = t2563 + t2565
  t2625 = cos(var1[16+1])
  t2634 = -1.0*t2625
  t2635 = 1.0 + t2634
  t2643 = sin(var1[16+1])
  t2375 = cos(var1[4+1])
  t2595 = -1.0*t2517*t2538
  t2596 = t2483*t2566
  t2601 = t2595 + t2596
  t2615 = t2483*t2538
  t2619 = t2517*t2566
  t2620 = t2615 + t2619
  t2637 = 0.051978134642000004*t2635
  t2696 = -0.05226439969100001*t2635
  t2664 = 0.49726168403800003*t2635
  t2744 = -0.073913*t2643
  t2669 = 0.994522*t2369*t2375*t2464
  t2673 = 0.103955395616*t2437*t2601
  t2676 = -0.9890740084840001*t2437
  t2677 = 1.0 + t2676
  t2683 = t2677*t2620
  t2684 = t2669 + t2673 + t2683
  t2736 = -0.703234*t2643
  t2710 = -0.104528*t2369*t2375*t2464
  t2716 = -0.010926102783999999*t2437
  t2721 = 1.0 + t2716
  t2722 = t2721*t2601
  t2724 = 0.103955395616*t2437*t2620
  t2725 = t2710 + t2722 + t2724
  t2750 = -1.0000001112680001*t2437
  t2753 = 1.0 + t2750
  t2754 = t2753*t2369*t2375
  t2755 = 0.104528*t2464*t2601
  t2756 = -0.994522*t2464*t2620
  t2759 = t2754 + t2755 + t2756
  t2767 = cos(var1[17+1])
  t2770 = -1.0*t2767
  t2771 = 1.0 + t2770
  t2773 = sin(var1[17+1])
  t2645 = -0.707107*t2643
  t2780 = -0.49726168403800003*t2635
  t2665 = 0.073913*t2643
  t2813 = -0.051978134642000004*t2635
  t2704 = 0.707107*t2643
  t2788 = 0.05226439969100001*t2635
  t2701 = 0.703234*t2643
  t2781 = t2780 + t2744
  t2782 = t2781*t2684
  t2791 = t2788 + t2736
  t2793 = t2791*t2725
  t2794 = -0.500001190325*t2635
  t2798 = 1.0 + t2794
  t2799 = t2798*t2759
  t2800 = t2782 + t2793 + t2799
  t2806 = -0.5054634410180001*t2635
  t2807 = 1.0 + t2806
  t2808 = t2807*t2684
  t2816 = t2813 + t2645
  t2819 = t2816*t2725
  t2821 = t2780 + t2665
  t2822 = t2821*t2759
  t2823 = t2808 + t2819 + t2822
  t2839 = t2813 + t2704
  t2840 = t2839*t2684
  t2844 = -0.9945383682050002*t2635
  t2845 = 1.0 + t2844
  t2846 = t2845*t2725
  t2847 = t2788 + t2701
  t2848 = t2847*t2759
  t2849 = t2840 + t2846 + t2848
  t2851 = -0.104528*t2773*t2800
  t2852 = 0.103955395616*t2771*t2823
  t2853 = -0.010926102783999999*t2771
  t2854 = 1.0 + t2853
  t2855 = t2854*t2849
  t2857 = t2851 + t2852 + t2855
  t2861 = 0.994522*t2773*t2800
  t2862 = -0.9890740084840001*t2771
  t2863 = 1.0 + t2862
  t2864 = t2863*t2823
  t2867 = 0.103955395616*t2771*t2849
  t2868 = t2861 + t2864 + t2867
  t2870 = -1.0000001112680001*t2771
  t2871 = 1.0 + t2870
  t2872 = t2871*t2800
  t2873 = -0.994522*t2773*t2823
  t2874 = 0.104528*t2773*t2849
  t2875 = t2872 + t2873 + t2874
  t2441 = -0.056500534356700764*t2437
  t2470 = -0.3852490428658858*t2464
  t2471 = t2441 + t2470
  t2512 = 0.4*t2506
  t2522 = 0.12*t2517
  t2523 = t2512 + t2522
  t2544 = 0.12*t2506
  t2545 = -0.4*t2517
  t2562 = t2544 + t2545
  t2571 = 1.1345904784751044e-7*var1[15+1]
  t2572 = -0.0402693119526853*t2437
  t2588 = 0.0059058871981009595*t2464
  t2592 = t2571 + t2572 + t2588
  t2892 = t2528*t2531*t2529
  t2893 = -1.0*t2369*t2532
  t2895 = t2892 + t2893
  t2897 = t2369*t2528
  t2900 = t2531*t2529*t2532
  t2901 = t2897 + t2900
  t2609 = 1.1924972351948546e-8*var1[15+1]
  t2612 = 0.3831386486090665*t2437
  t2613 = -0.05619101817723254*t2464
  t2614 = t2609 + t2612 + t2613
  t2622 = -4.0332087336819504e-7*var1[16+1]
  t2636 = 0.0958179942122405*t2635
  t2650 = t2637 + t2645
  t2654 = 0.23105307644*t2650
  t2666 = t2664 + t2665
  t2667 = 0.164374659834*t2666
  t2668 = t2622 + t2636 + t2654 + t2667
  t2904 = -1.0*t2517*t2895
  t2905 = t2483*t2901
  t2906 = t2904 + t2905
  t2908 = t2483*t2895
  t2909 = t2517*t2901
  t2910 = t2908 + t2909
  t2686 = 4.239080549754904e-8*var1[16+1]
  t2695 = 0.22979114961138278*t2635
  t2702 = t2696 + t2701
  t2703 = 0.164374659834*t2702
  t2705 = t2637 + t2704
  t2706 = 0.189564637987*t2705
  t2707 = t2686 + t2695 + t2703 + t2706
  t2730 = 4.05542127947119e-7*var1[16+1]
  t2735 = 0.08218752557626696*t2635
  t2737 = t2696 + t2736
  t2738 = 0.23105307644*t2737
  t2745 = t2664 + t2744
  t2748 = 0.189564637987*t2745
  t2749 = t2730 + t2735 + t2738 + t2748
  t2772 = 0.19098732144477495*t2771
  t2774 = -0.13776101532839094*t2773
  t2779 = t2772 + t2774
  t2916 = 0.994522*t2375*t2464*t2531
  t2917 = 0.103955395616*t2437*t2906
  t2918 = t2677*t2910
  t2919 = t2916 + t2917 + t2918
  t2922 = -0.104528*t2375*t2464*t2531
  t2923 = t2721*t2906
  t2925 = 0.103955395616*t2437*t2910
  t2926 = t2922 + t2923 + t2925
  t2930 = t2753*t2375*t2531
  t2931 = 0.104528*t2464*t2906
  t2934 = -0.994522*t2464*t2910
  t2936 = t2930 + t2931 + t2934
  t2802 = -5.06291820800569e-8*var1[17+1]
  t2803 = 0.13700636048642204*t2771
  t2804 = 0.18994107176353728*t2773
  t2805 = t2802 + t2803 + t2804
  t2830 = -4.817066759205414e-7*var1[17+1]
  t2833 = -0.014399883410246048*t2771
  t2835 = -0.019963520514678434*t2773
  t2837 = t2830 + t2833 + t2835
  t2938 = t2781*t2919
  t2939 = t2791*t2926
  t2940 = t2798*t2936
  t2941 = t2938 + t2939 + t2940
  t2943 = t2807*t2919
  t2944 = t2816*t2926
  t2945 = t2821*t2936
  t2946 = t2943 + t2944 + t2945
  t2949 = t2839*t2919
  t2950 = t2845*t2926
  t2952 = t2847*t2936
  t2954 = t2949 + t2950 + t2952
  t2958 = -0.104528*t2773*t2941
  t2959 = 0.103955395616*t2771*t2946
  t2960 = t2854*t2954
  t2961 = t2958 + t2959 + t2960
  t2963 = 0.994522*t2773*t2941
  t2965 = t2863*t2946
  t2966 = 0.103955395616*t2771*t2954
  t2968 = t2963 + t2965 + t2966
  t2972 = t2871*t2941
  t2973 = -0.994522*t2773*t2946
  t2974 = 0.104528*t2773*t2954
  t2975 = t2972 + t2973 + t2974
  t3004 = -1.0*t2375*t2528*t2517
  t3006 = t2483*t2375*t2532
  t3007 = t3004 + t3006
  t3009 = t2483*t2375*t2528
  t3010 = t2375*t2517*t2532
  t3011 = t3009 + t3010
  t3013 = -0.994522*t2464*t2529
  t3014 = 0.103955395616*t2437*t3007
  t3015 = t2677*t3011
  t3016 = t3013 + t3014 + t3015
  t3021 = 0.104528*t2464*t2529
  t3022 = t2721*t3007
  t3023 = 0.103955395616*t2437*t3011
  t3024 = t3021 + t3022 + t3023
  t3026 = -1.0*t2753*t2529
  t3027 = 0.104528*t2464*t3007
  t3028 = -0.994522*t2464*t3011
  t3029 = t3026 + t3027 + t3028
  t3031 = t2781*t3016
  t3032 = t2791*t3024
  t3035 = t2798*t3029
  t3036 = t3031 + t3032 + t3035
  t3038 = t2807*t3016
  t3039 = t2816*t3024
  t3041 = t2821*t3029
  t3042 = t3038 + t3039 + t3041
  t3044 = t2839*t3016
  t3045 = t2845*t3024
  t3046 = t2847*t3029
  t3047 = t3044 + t3045 + t3046
  t3052 = -0.104528*t2773*t3036
  t3053 = 0.103955395616*t2771*t3042
  t3054 = t2854*t3047
  t3056 = t3052 + t3053 + t3054
  t3059 = 0.994522*t2773*t3036
  t3060 = t2863*t3042
  t3061 = 0.103955395616*t2771*t3047
  t3062 = t3059 + t3060 + t3061
  t3065 = t2871*t3036
  t3066 = -0.994522*t2773*t3042
  t3069 = 0.104528*t2773*t3047
  t3070 = t3065 + t3066 + t3069
  p_output1[0+1]=t2369*t2375*t2471 + t2523*t2538 + t2562*t2566 + t2592*t2601 + t2614*t2620 + t2668*t2684 + t2707*t2725 + t2749*t2759 + t2779*t2800 + t2805*t2823 + t2837*t2849 + 0.272124*t2857 - 0.07912*(0.994522*t2857 + 0.104528*t2868) + 0.167122*t2868 + 0.369*(-0.040001*t2857 + 0.380588*t2868 + 0.92388*t2875) + 0.190987*t2875
  p_output1[1+1]=t2375*t2471*t2531 + t2523*t2895 + t2562*t2901 + t2592*t2906 + t2614*t2910 + t2668*t2919 + t2707*t2926 + t2749*t2936 + t2779*t2941 + t2805*t2946 + t2837*t2954 + 0.272124*t2961 - 0.07912*(0.994522*t2961 + 0.104528*t2968) + 0.167122*t2968 + 0.369*(-0.040001*t2961 + 0.380588*t2968 + 0.92388*t2975) + 0.190987*t2975
  p_output1[2+1]=t2375*t2523*t2528 - 1.0*t2471*t2529 + t2375*t2532*t2562 + t2592*t3007 + t2614*t3011 + t2668*t3016 + t2707*t3024 + t2749*t3029 + t2779*t3036 + t2805*t3042 + t2837*t3047 + 0.272124*t3056 - 0.07912*(0.994522*t3056 + 0.104528*t3062) + 0.167122*t3062 + 0.369*(-0.040001*t3056 + 0.380588*t3062 + 0.92388*t3070) + 0.190987*t3070
end



function p_left_hand_wrt_base(θ::AbstractVector{T}) where T
  left_hand = zeros(T, 3)
  p_left_hand_wrt_base_helper!(left_hand, θ)
  return left_hand
end
