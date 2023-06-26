echo ""
echo "    Executing getSequences.sh ..."
echo "        dataset = $dataset"
echo "        sequenceGroup = $sequenceGroup"
echo ""

if [ $dataset == 'rgbdtum' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"rgbd_dataset_freiburg1_desk" 
			"rgbd_dataset_freiburg1_floor" 
			"rgbd_dataset_freiburg1_xyz"
			"rgbd_dataset_freiburg2_360_kidnap"
			"rgbd_dataset_freiburg2_desk"
			"rgbd_dataset_freiburg2_desk_with_person"
			"rgbd_dataset_freiburg2_xyz"
			"rgbd_dataset_freiburg3_long_office_household"
			"rgbd_dataset_freiburg3_nostructure_texture_far"
			"rgbd_dataset_freiburg3_nostructure_texture_near_withloop"
			"rgbd_dataset_freiburg3_sitting_halfsphere"
			"rgbd_dataset_freiburg3_sitting_xyz"
			"rgbd_dataset_freiburg3_structure_texture_far"
			"rgbd_dataset_freiburg3_structure_texture_near"
			"rgbd_dataset_freiburg3_walking_halfsphere"
			"rgbd_dataset_freiburg3_walking_xyz"
		)
		sequenceSettings=(
			"rgbdtum1.yaml" "rgbdtum1.yaml" "rgbdtum1.yaml"
			"rgbdtum2.yaml" "rgbdtum2.yaml" "rgbdtum2.yaml" "rgbdtum2.yaml"
			"rgbdtum3.yaml" "rgbdtum3.yaml" "rgbdtum3.yaml"
			"rgbdtum3.yaml" "rgbdtum3.yaml" "rgbdtum3.yaml"
			"rgbdtum3.yaml" "rgbdtum3.yaml" "rgbdtum3.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'testGroup' ]
	then 
		sequenceNames=(
			"rgbd_dataset_freiburg1_desk" 
			"rgbd_dataset_freiburg1_xyz"
			"rgbd_dataset_freiburg2_desk"
			"rgbd_dataset_freiburg2_xyz"
			"rgbd_dataset_freiburg3_long_office_household"
			"rgbd_dataset_freiburg3_nostructure_texture_near_withloop"
			"rgbd_dataset_freiburg3_structure_texture_far"
			"rgbd_dataset_freiburg3_structure_texture_near"
		)
		sequenceSettings=(
			"rgbdtum1.yaml" "rgbdtum1.yaml"
			"rgbdtum2.yaml" "rgbdtum2.yaml"
			"rgbdtum3.yaml" "rgbdtum3.yaml" "rgbdtum3.yaml" "rgbdtum3.yaml" 
		)
	fi
	
	if [ $sequenceGroup == 'fr1' ]
	then 
		sequenceNames=(
			"rgbd_dataset_freiburg1_desk" 
			"rgbd_dataset_freiburg1_floor" 
			"rgbd_dataset_freiburg1_xyz"
		)
		sequenceSettings=(
			"rgbdtum1.yaml" "rgbdtum1.yaml" "rgbdtum1.yaml"			
		)
	fi
	
	if [ $sequenceGroup == 'fr2' ]
	then 
		sequenceNames=(			
			"rgbd_dataset_freiburg2_360_kidnap"
			"rgbd_dataset_freiburg2_desk"
			"rgbd_dataset_freiburg2_desk_with_person"
			"rgbd_dataset_freiburg2_xyz"			
		)
		sequenceSettings=(			
			"rgbdtum2.yaml" "rgbdtum2.yaml" "rgbdtum2.yaml" "rgbdtum2.yaml"			
		)
	fi
	
	if [ $sequenceGroup == 'fr3' ]
	then 
		sequenceNames=(			
			"rgbd_dataset_freiburg3_long_office_household"
			"rgbd_dataset_freiburg3_nostructure_texture_far"
			"rgbd_dataset_freiburg3_nostructure_texture_near_withloop"
			"rgbd_dataset_freiburg3_sitting_halfsphere"
			"rgbd_dataset_freiburg3_sitting_xyz"
			"rgbd_dataset_freiburg3_structure_texture_far"
			"rgbd_dataset_freiburg3_structure_texture_near"
			"rgbd_dataset_freiburg3_walking_halfsphere"
			"rgbd_dataset_freiburg3_walking_xyz"
		)
		sequenceSettings=(			
			"rgbdtum3.yaml" "rgbdtum3.yaml" "rgbdtum3.yaml"
			"rgbdtum3.yaml" "rgbdtum3.yaml" "rgbdtum3.yaml"
			"rgbdtum3.yaml" "rgbdtum3.yaml" "rgbdtum3.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'threeSequences' ]
	then 
		sequenceNames=(
			"rgbd_dataset_freiburg1_xyz"			
			"rgbd_dataset_freiburg2_desk"
			"rgbd_dataset_freiburg3_long_office_household"
		)
		sequenceSettings=(
			"rgbdtum1.yaml"
			"rgbdtum2.yaml"
			"rgbdtum3.yaml"			
		)
	fi
	
	if [ $sequenceGroup == 'rgbd_dataset_freiburg1_desk' ]
	then 
		sequenceNames=("rgbd_dataset_freiburg1_desk")
		sequenceSettings=("rgbdtum1.yaml")
	fi
	
	if [ $sequenceGroup == 'rgbd_dataset_freiburg1_xyz' ]
	then 
		sequenceNames=("rgbd_dataset_freiburg1_xyz")
		sequenceSettings=("rgbdtum1.yaml")
	fi
	
	if [ $sequenceGroup == 'rgbd_dataset_freiburg2_xyz' ]
	then 
		sequenceNames=("rgbd_dataset_freiburg2_xyz")
		sequenceSettings=("rgbdtum2.yaml")
	fi
	
	if [ $sequenceGroup == 'rgbd_dataset_freiburg2_desk' ]
	then 
		sequenceNames=("rgbd_dataset_freiburg2_desk")
		sequenceSettings=("rgbdtum2.yaml")
	fi
	
	if [ $sequenceGroup == 'rgbd_dataset_freiburg3_structure_texture_far' ]
	then 
		sequenceNames=("rgbd_dataset_freiburg3_structure_texture_far")
		sequenceSettings=("rgbdtum3.yaml")
	fi
	
	if [ $sequenceGroup == 'rgbd_dataset_freiburg3_structure_texture_near' ]
	then 
		sequenceNames=("rgbd_dataset_freiburg3_structure_texture_near")
		sequenceSettings=("rgbdtum3.yaml")
	fi
	
	if [ $sequenceGroup == 'rgbd_dataset_freiburg3_long_office_household' ]
	then 
		sequenceNames=("rgbd_dataset_freiburg3_long_office_household")
		sequenceSettings=("rgbdtum3.yaml")
	fi
	
	if [ $sequenceGroup == 'rgbd_dataset_freiburg3_nostructure_texture_near_withloop' ]
	then 
		sequenceNames=("rgbd_dataset_freiburg3_nostructure_texture_near_withloop")
		sequenceSettings=("rgbdtum3.yaml")
	fi	
				
	if [ $sequenceGroup == 'rgbd_dataset_freiburg3_nostructure_texture_far' ]
	then 
		sequenceNames=("rgbd_dataset_freiburg3_nostructure_texture_far")
		sequenceSettings=("rgbdtum3.yaml")
	fi
	
	if [ $sequenceGroup == 'SP' ]
	then 
		sequenceNames=(
			"rgbd_dataset_freiburg1_xyz"
			"rgbd_dataset_freiburg2_desk"
			"rgbd_dataset_freiburg2_xyz"
			"rgbd_dataset_freiburg3_long_office_household"
			"rgbd_dataset_freiburg3_nostructure_texture_near_withloop"
			"rgbd_dataset_freiburg3_structure_texture_far"
		)
		sequenceSettings=(
			"rgbdtum1.yaml"
			"rgbdtum2.yaml" "rgbdtum2.yaml" 
			"rgbdtum3.yaml" "rgbdtum3.yaml" "rgbdtum3.yaml"
		)
	fi
fi

if [ $dataset == 'monotum' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"sequence_01" "sequence_02" "sequence_03" "sequence_04" "sequence_05" "sequence_06" "sequence_07" "sequence_08" "sequence_09" "sequence_10" 
			"sequence_11" "sequence_12" "sequence_13" "sequence_14" "sequence_15" "sequence_16" "sequence_17" "sequence_18" "sequence_19" "sequence_20" 
			"sequence_21" "sequence_22" "sequence_23" "sequence_24" "sequence_25" "sequence_26" "sequence_27" "sequence_28" "sequence_29" "sequence_30" 
			"sequence_31" "sequence_32" "sequence_33" "sequence_34" "sequence_35" "sequence_36" "sequence_37" "sequence_38" "sequence_39" "sequence_40" 
			"sequence_41" "sequence_42" "sequence_43" "sequence_44" "sequence_45" "sequence_46" "sequence_47" "sequence_48" "sequence_49" "sequence_50" 
		)
		sequenceSettings=(
			"monotum1.yaml" "monotum1.yaml" "monotum1.yaml" "monotum1.yaml" "monotum1.yaml" "monotum1.yaml"
			"monotum2.yaml" "monotum2.yaml"
			"monotum1.yaml" "monotum1.yaml" "monotum1.yaml" "monotum1.yaml" "monotum1.yaml" "monotum1.yaml" "monotum1.yaml"
			"monotum2.yaml"
			"monotum1.yaml" "monotum1.yaml" "monotum1.yaml" "monotum1.yaml" "monotum1.yaml" "monotum1.yaml"
			"monotum2.yaml" "monotum2.yaml" "monotum2.yaml" "monotum2.yaml"
			"monotum1.yaml" 
			"monotum2.yaml" "monotum2.yaml" "monotum2.yaml" "monotum2.yaml" "monotum2.yaml"
			"monotum1.yaml" "monotum1.yaml" 
			"monotum2.yaml" "monotum2.yaml" "monotum2.yaml" "monotum2.yaml" "monotum2.yaml" "monotum2.yaml" "monotum2.yaml" "monotum2.yaml" 
			"monotum2.yaml" "monotum2.yaml" "monotum2.yaml" "monotum2.yaml" "monotum2.yaml" "monotum2.yaml" "monotum2.yaml" "monotum2.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'testGroup' ]
	then 
		sequenceNames=(
			"sequence_01" "sequence_11" "sequence_21" "sequence_31" "sequence_41"
		)
		sequenceSettings=(
			"monotum1.yaml" "monotum1.yaml" "monotum1.yaml" "monotum2.yaml" "monotum2.yaml"
		)		
	fi
	
	if [ $sequenceGroup == 'motionBiasSubset' ]
	then 
		sequenceNames=(
			"sequence_17" "sequence_23" "sequence_29" "sequence_46" "sequence_47"
		)
		sequenceSettings=(
			"monotum1.yaml" "monotum2.yaml" "monotum2.yaml" "monotum2.yaml" "monotum2.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'sequence_01' ]
	then 
		sequenceNames=("sequence_01")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_02' ]
	then 
		sequenceNames=("sequence_02")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_03' ]
	then 
		sequenceNames=("sequence_03")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_04' ]
	then 
		sequenceNames=("sequence_04")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_05' ]
	then 
		sequenceNames=("sequence_05")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_06' ]
	then 
		sequenceNames=("sequence_06")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_07' ]
	then 
		sequenceNames=("sequence_07")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_08' ]
	then 
		sequenceNames=("sequence_08")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_09' ]
	then 
		sequenceNames=("sequence_09")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_10' ]
	then 
		sequenceNames=("sequence_10")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_11' ]
	then 
		sequenceNames=("sequence_11")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_12' ]
	then 
		sequenceNames=("sequence_12")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_13' ]
	then 
		sequenceNames=("sequence_13")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_14' ]
	then 
		sequenceNames=("sequence_14")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_15' ]
	then 
		sequenceNames=("sequence_15")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_16' ]
	then 
		sequenceNames=("sequence_16")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_17' ]
	then 
		sequenceNames=("sequence_17")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_18' ]
	then 
		sequenceNames=("sequence_18")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_19' ]
	then 
		sequenceNames=("sequence_19")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_20' ]
	then 
		sequenceNames=("sequence_20")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_21' ]
	then 
		sequenceNames=("sequence_21")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_22' ]
	then 
		sequenceNames=("sequence_22")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_23' ]
	then 
		sequenceNames=("sequence_23")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_24' ]
	then 
		sequenceNames=("sequence_24")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_25' ]
	then 
		sequenceNames=("sequence_25")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_26' ]
	then 
		sequenceNames=("sequence_26")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_27' ]
	then 
		sequenceNames=("sequence_27")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_28' ]
	then 
		sequenceNames=("sequence_28")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_29' ]
	then 
		sequenceNames=("sequence_29")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_30' ]
	then 
		sequenceNames=("sequence_30")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_31' ]
	then 
		sequenceNames=("sequence_31")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_32' ]
	then 
		sequenceNames=("sequence_32")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_33' ]
	then 
		sequenceNames=("sequence_33")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_34' ]
	then 
		sequenceNames=("sequence_34")
		sequenceSettings=("monotum1.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_35' ]
	then 
		sequenceNames=("sequence_35")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_36' ]
	then 
		sequenceNames=("sequence_36")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_37' ]
	then 
		sequenceNames=("sequence_37")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_38' ]
	then 
		sequenceNames=("sequence_38")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_39' ]
	then 
		sequenceNames=("sequence_39")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_40' ]
	then 
		sequenceNames=("sequence_40")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_41' ]
	then 
		sequenceNames=("sequence_41")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_42' ]
	then 
		sequenceNames=("sequence_42")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_43' ]
	then 
		sequenceNames=("sequence_43")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_44' ]
	then 
		sequenceNames=("sequence_44")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_45' ]
	then 
		sequenceNames=("sequence_45")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_46' ]
	then 
		sequenceNames=("sequence_46")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_47' ]
	then 
		sequenceNames=("sequence_47")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_48' ]
	then 
		sequenceNames=("sequence_48")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_49' ]
	then 
		sequenceNames=("sequence_49")
		sequenceSettings=("monotum2.yaml")
	fi
	
	if [ $sequenceGroup == 'sequence_50' ]
	then 
		sequenceNames=("sequence_50")
		sequenceSettings=("monotum2.yaml")
	fi
	
fi

if [ $dataset == 'kitti' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"00" "01" "02" "03" "04" "05" "06" "07" "08" "09" "10"
		)
		sequenceSettings=( 
			"KITTI00-02.yaml" "KITTI00-02.yaml" "KITTI00-02.yaml"
			"KITTI03.yaml"
			"KITTI04-12.yaml" "KITTI04-12.yaml" "KITTI04-12.yaml" "KITTI04-12.yaml"
			"KITTI04-12.yaml" "KITTI04-12.yaml" "KITTI04-12.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'testGroup' ]
	then 
		sequenceNames=(
			"00" "01" "02" "03" "04" "05" "06" "07" "08" "09" "10"
		)
		sequenceSettings=( 
			"KITTI00-02.yaml" "KITTI00-02.yaml" "KITTI00-02.yaml"
			"KITTI03.yaml"
			"KITTI04-12.yaml" "KITTI04-12.yaml" "KITTI04-12.yaml" "KITTI04-12.yaml"
			"KITTI04-12.yaml" "KITTI04-12.yaml" "KITTI04-12.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'motionBiasSubset' ]
	then 
		sequenceNames=(
			"03" "04" "06" "07"
		)
		sequenceSettings=( 			
			"KITTI03.yaml"
			"KITTI04-12.yaml" "KITTI04-12.yaml" "KITTI04-12.yaml" 
		)
	fi
	
	if [ $sequenceGroup == '00' ]
	then 
		sequenceNames=("00")
		sequenceSettings=("KITTI00-02.yaml")
	fi
	
	if [ $sequenceGroup == '01' ]
	then 
		sequenceNames=("01")
		sequenceSettings=("KITTI00-02.yaml")
	fi
	
	if [ $sequenceGroup == '02' ]
	then 
		sequenceNames=("02")
		sequenceSettings=("KITTI00-02.yaml")
	fi
	
	if [ $sequenceGroup == '03' ]
	then 
		sequenceNames=("03")
		sequenceSettings=("KITTI03.yaml")
	fi
	
	if [ $sequenceGroup == '04' ]
	then 
		sequenceNames=("04")
		sequenceSettings=("KITTI04-12.yaml")
	fi
	
	if [ $sequenceGroup == '05' ]
	then 
		sequenceNames=("05")
		sequenceSettings=("KITTI04-12.yaml")
	fi
	
	if [ $sequenceGroup == '06' ]
	then 
		sequenceNames=("06")
		sequenceSettings=("KITTI04-12.yaml")
	fi
	
	if [ $sequenceGroup == '07' ]
	then 
		sequenceNames=("07")
		sequenceSettings=("KITTI04-12.yaml")
	fi
	
	if [ $sequenceGroup == '08' ]
	then 
		sequenceNames=("08")
		sequenceSettings=("KITTI04-12.yaml")
	fi
	
	if [ $sequenceGroup == '09' ]
	then 
		sequenceNames=("09")
		sequenceSettings=("KITTI04-12.yaml")
	fi
	
	if [ $sequenceGroup == '10' ]
	then 
		sequenceNames=("10")
		sequenceSettings=("KITTI04-12.yaml")
	fi
fi

if [ $dataset == 'fordva' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"2017-10-26-V2-Log1-FL"
		)
		sequenceSettings=( 
			"monofordva.yaml"
		)
	fi
fi

if [ $dataset == 'madmax' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"C-0"
		)
		sequenceSettings=( 
			"madmax_c.yaml"
		)
	fi
fi

if [ $dataset == 'euroc' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"V1_01_easy" "V1_02_medium" "V1_03_difficult"
			"V2_01_easy" "V2_02_medium" "V2_03_difficult"
			"MH_01_easy" "MH_02_easy" "MH_03_medium" "MH_04_difficult" "MH_05_difficult"
		)
		sequenceSettings="EuRoC.yaml"
		sequenceTimestamps=(
			"V101" "V102" "V103"
			"V201" "V202" "V203"
			"MH01" "MH02" "MH03"
			"MH04" "MH05" 
		)		
	fi
	
	if [ $sequenceGroup == 'testGroup' ]
	then 
		sequenceNames=(
			"V1_01_easy" "V1_02_medium" "V1_03_difficult"
			"V2_01_easy" "V2_02_medium" "V2_03_difficult"
			"MH_01_easy" "MH_02_easy" "MH_03_medium" "MH_04_difficult" "MH_05_difficult"
		)
		sequenceSettings="EuRoC.yaml"
		sequenceTimestamps=(
			"V101" "V102" "V103"
			"V201" "V202" "V203"
			"MH01" "MH02" "MH03"
			"MH04" "MH05" 
		)
	fi
	
	if [ $sequenceGroup == 'V1_01_easy' ]
	then 
		sequenceNames=("V1_01_easy")
		sequenceSettings="EuRoC.yaml"
		sequenceTimestamps=("V101")		
	fi
	
	if [ $sequenceGroup == 'V1_02_medium' ]
	then 
		sequenceNames=("V1_02_medium")
		sequenceSettings="EuRoC.yaml"
		sequenceTimestamps=("V102")		
	fi
	
	if [ $sequenceGroup == 'V1_03_difficult' ]
	then 
		sequenceNames=("V1_03_difficult")
		sequenceSettings="EuRoC.yaml"
		sequenceTimestamps=("V103")		
	fi
	
	if [ $sequenceGroup == 'V2_01_easy' ]
	then 
		sequenceNames=("V2_01_easy")
		sequenceSettings="EuRoC.yaml"
		sequenceTimestamps=("V201")		
	fi
	
	if [ $sequenceGroup == 'V2_02_medium' ]
	then 
		sequenceNames=("V2_02_medium")
		sequenceSettings="EuRoC.yaml"
		sequenceTimestamps=("V202")		
	fi
	
	if [ $sequenceGroup == 'V2_03_difficult' ]
	then 
		sequenceNames=("V2_03_difficult")
		sequenceSettings="EuRoC.yaml"
		sequenceTimestamps=("V203")		
	fi
	
	if [ $sequenceGroup == 'MH_01_easy' ]
	then 
		sequenceNames=("MH_01_easy")
		sequenceSettings="EuRoC.yaml"
		sequenceTimestamps=("MH01")		
	fi
	
	if [ $sequenceGroup == 'MH_02_easy' ]
	then 
		sequenceNames=("MH_02_easy")
		sequenceSettings="EuRoC.yaml"
		sequenceTimestamps=("MH02")		
	fi
	
	if [ $sequenceGroup == 'MH_03_medium' ]
	then 
		sequenceNames=("MH_03_medium")
		sequenceSettings="EuRoC.yaml"
		sequenceTimestamps=("MH03")		
	fi
	
	if [ $sequenceGroup == 'MH_04_difficult' ]
	then 
		sequenceNames=("MH_04_difficult")
		sequenceSettings="EuRoC.yaml"
		sequenceTimestamps=("MH04")		
	fi
	
	if [ $sequenceGroup == 'MH_05_difficult' ]
	then 
		sequenceNames=("MH_05_difficult")
		sequenceSettings="EuRoC.yaml"
		sequenceTimestamps=("MH05")		
	fi
fi

if [ $dataset == 'oxford' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"exp04_construction_upper_level"
			"exp21_outside_building"
		)
		sequenceSettings=( 
			"oxford0.yaml"
			"oxford0.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'exp04_construction_upper_level' ]
	then 
		sequenceNames=(
			"exp04_construction_upper_level"
		)
		sequenceSettings=( 
			"oxford0.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'exp21_outside_building' ]
	then 
		sequenceNames=(
			"exp21_outside_building"
		)
		sequenceSettings=( 
			"oxford0.yaml"
		)
	fi
fi

if [ $dataset == 'vector' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"school_scooter1"
			
		)
		sequenceSettings=( 
			"vector_right.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'school_scooter1' ]
	then 
		sequenceNames=(
			"school_scooter1"
		)
		sequenceSettings=( 
			"vector_right.yaml"
		)
	fi
	
fi
