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
	
	if [ $sequenceGroup == 'miniTestGroup' ]
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


if [ $dataset == 'nuim' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"living_room_traj0_frei_png" 
			"living_room_traj1_frei_png" 
			"living_room_traj2_frei_png" 
			"living_room_traj3_frei_png" 
			"traj0_frei_png"
			"traj1_frei_png"
			"traj2_frei_png"
			"traj3_frei_png"
			
		)
		sequenceSettings=(
			"nuim.yaml" "nuim.yaml" "nuim.yaml" "nuim.yaml"
			"nuim.yaml" "nuim.yaml" "nuim.yaml" "nuim.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'miniTestGroup' ]
	then 
		sequenceNames=(
			"living_room_traj0_frei_png" 
			"living_room_traj1_frei_png" 
			"living_room_traj2_frei_png" 
			"living_room_traj3_frei_png" 
			"traj0_frei_png"
			"traj1_frei_png"
			"traj2_frei_png"
			"traj3_frei_png"
			
		)
		sequenceSettings=(
			"nuim.yaml" "nuim.yaml" "nuim.yaml" "nuim.yaml"
			"nuim.yaml" "nuim.yaml" "nuim.yaml" "nuim.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'living_room_traj0_frei_png' ]
	then 
		sequenceNames=("living_room_traj0_frei_png")
		sequenceSettings=("nuim.yaml")
	fi
	
	if [ $sequenceGroup == 'living_room_traj1_frei_png' ]
	then 
		sequenceNames=("living_room_traj1_frei_png")
		sequenceSettings=("nuim.yaml")
	fi
	
	if [ $sequenceGroup == 'living_room_traj2_frei_png' ]
	then 
		sequenceNames=("living_room_traj2_frei_png")
		sequenceSettings=("nuim.yaml")
	fi
	
	if [ $sequenceGroup == 'living_room_traj3_frei_png' ]
	then 
		sequenceNames=("living_room_traj3_frei_png")
		sequenceSettings=("nuim.yaml")
	fi
	
	if [ $sequenceGroup == 'traj0_frei_png' ]
	then 
		sequenceNames=("traj0_frei_png")
		sequenceSettings=("nuim.yaml")
	fi
	
	if [ $sequenceGroup == 'traj1_frei_png' ]
	then 
		sequenceNames=("traj1_frei_png")
		sequenceSettings=("nuim.yaml")
	fi		
	
	if [ $sequenceGroup == 'traj2_frei_png' ]
	then 
		sequenceNames=("traj2_frei_png")
		sequenceSettings=("nuim.yaml")
	fi
	
	if [ $sequenceGroup == 'traj3_frei_png' ]
	then 
		sequenceNames=("traj3_frei_png")
		sequenceSettings=("nuim.yaml")
	fi	
fi

if [ $dataset == 'tartanair' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"ME000" "ME001" "ME002" "ME003" "ME004" "ME005" "ME006" "ME007" 
			"MH000" "MH001" "MH002" "MH003" "MH004" "MH005" "MH006" "MH007"  				
		)
		sequenceSettings=(
			"tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml"
			"tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'easy' ]
	then 
		sequenceNames=(
			"ME000" "ME001" "ME002" "ME003" "ME004" "ME005" "ME006" "ME007" 
		)
		sequenceSettings=(
			"tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'hard' ]
	then 
		sequenceNames=(
			"MH000" "MH001" "MH002" "MH003" "MH004" "MH005" "MH006" "MH007"  				
		)
		sequenceSettings=(
			"tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml" "tartanair.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'ME000' ]
	then 
		sequenceNames=("ME000")
		sequenceSettings=("tartanair.yaml")
	fi
	if [ $sequenceGroup == 'ME001' ]
	then 
		sequenceNames=("ME001")
		sequenceSettings=("tartanair.yaml")
	fi
	if [ $sequenceGroup == 'ME002' ]
	then 
		sequenceNames=("ME002")
		sequenceSettings=("tartanair.yaml")
	fi
	if [ $sequenceGroup == 'ME003' ]
	then 
		sequenceNames=("ME003")
		sequenceSettings=("tartanair.yaml")
	fi
	if [ $sequenceGroup == 'ME004' ]
	then 
		sequenceNames=("ME004")
		sequenceSettings=("tartanair.yaml")
	fi
	if [ $sequenceGroup == 'ME005' ]
	then 
		sequenceNames=("ME005")
		sequenceSettings=("tartanair.yaml")
	fi
	if [ $sequenceGroup == 'ME006' ]
	then 
		sequenceNames=("ME006")
		sequenceSettings=("tartanair.yaml")
	fi
	if [ $sequenceGroup == 'ME007' ]
	then 
		sequenceNames=("ME007")
		sequenceSettings=("tartanair.yaml")
	fi
	if [ $sequenceGroup == 'MH000' ]
	then 
		sequenceNames=("MH000")
		sequenceSettings=("tartanair.yaml")
	fi
	if [ $sequenceGroup == 'MH001' ]
	then 
		sequenceNames=("MH001")
		sequenceSettings=("tartanair.yaml")
	fi
	if [ $sequenceGroup == 'MH002' ]
	then 
		sequenceNames=("MH002")
		sequenceSettings=("tartanair.yaml")
	fi
	if [ $sequenceGroup == 'MH003' ]
	then 
		sequenceNames=("MH003")
		sequenceSettings=("tartanair.yaml")
	fi
	if [ $sequenceGroup == 'MH004' ]
	then 
		sequenceNames=("MH004")
		sequenceSettings=("tartanair.yaml")
	fi
	if [ $sequenceGroup == 'MH005' ]
	then 
		sequenceNames=("MH005")
		sequenceSettings=("tartanair.yaml")
	fi
	if [ $sequenceGroup == 'MH006' ]
	then 
		sequenceNames=("MH006")
		sequenceSettings=("tartanair.yaml")
	fi
	if [ $sequenceGroup == 'MH007' ]
	then 
		sequenceNames=("MH007")
		sequenceSettings=("tartanair.yaml")
	fi
	
fi

if [ $dataset == 'drunkards' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"00000_level0" 
			"00000_level2" 			
		)
		sequenceSettings=(
			"drunkards.yaml" 
			"drunkards.yaml" 
		)
	fi

	if [ $sequenceGroup == '00000_level0' ]
	then 
		sequenceNames=("00000_level0")
		sequenceSettings=("drunkards.yaml")
	fi
	if [ $sequenceGroup == '00000_level2' ]
	then 
		sequenceNames=("00000_level2")
		sequenceSettings=("drunkards.yaml")
	fi

fi

if [ $dataset == 'rosario' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"sequence01" 
		)
		sequenceSettings=(
			"rosario.yaml" 
		)
	fi

	if [ $sequenceGroup == 'sequence01' ]
	then 
		sequenceNames=("sequence01")
		sequenceSettings=("rosario.yaml")
	fi
fi

if [ $dataset == 'openloris' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"office1-1" 
			"office1-2"
			"cafe1-1" 
			"cafe1-2"
		)
		sequenceSettings=(
			"openloris.yaml" 
			"openloris.yaml" 
			"openloris.yaml" 
			"openloris.yaml" 
		)
	fi

	if [ $sequenceGroup == 'miniTestGroup' ]
	then 
		sequenceNames=(
			"office1-1" 
			"office1-2"
			"cafe1-1" 
			"cafe1-2"
		)
		sequenceSettings=(
			"openloris.yaml" 
			"openloris.yaml" 
			"openloris.yaml" 
			"openloris.yaml" 
		)
	fi
	
	if [ $sequenceGroup == 'office1-1' ]
	then 
		sequenceNames=("office1-1")
		sequenceSettings=("openloris.yaml")
	fi
	
	if [ $sequenceGroup == 'office1-2' ]
	then 
		sequenceNames=("office1-2")
		sequenceSettings=("openloris.yaml")
	fi
	
	if [ $sequenceGroup == 'cafe1-1' ]
	then 
		sequenceNames=("cafe1-1")
		sequenceSettings=("openloris.yaml")
	fi
	
	if [ $sequenceGroup == 'cafe1-2' ]
	then 
		sequenceNames=("cafe1-2")
		sequenceSettings=("openloris.yaml")
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
	
	
	if [ $sequenceGroup == 'miniTestGroup' ]
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
			"00" "02" "03" "04" "05" "06" "07" "08" "09" "10"
		)
		sequenceSettings=( 
			"KITTI00-02.yaml" "KITTI00-02.yaml"
			"KITTI03.yaml"
			"KITTI04-12.yaml" "KITTI04-12.yaml" "KITTI04-12.yaml" "KITTI04-12.yaml"
			"KITTI04-12.yaml" "KITTI04-12.yaml" "KITTI04-12.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'miniTestGroup' ]
	then 
		sequenceNames=(
			"00" "02" "03" "04" "05" "06" "07" "08" "09" "10"
		)
		sequenceSettings=( 
			"KITTI00-02.yaml" "KITTI00-02.yaml"
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


if [ $dataset == 'vkitti' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"Scene01_morning","Scene02_morning","Scene06_morning","Scene18_morning","Scene20_morning"
		)
		sequenceSettings=( 
			"vkitti.yaml" "vkitti.yaml" "vkitti.yaml" "vkitti.yaml" "vkitti.yaml"		
		)
	fi
	
	if [ $sequenceGroup == 'miniTestGroup' ]
	then 
		sequenceNames=(
			"Scene01_morning","Scene02_morning","Scene06_morning","Scene18_morning","Scene20_morning"
		)
		sequenceSettings=( 
			"vkitti.yaml" "vkitti.yaml" "vkitti.yaml" "vkitti.yaml" "vkitti.yaml"		
		)
	fi
	
	
	if [ $sequenceGroup == 'Scene01_morning' ]
	then 
		sequenceNames=("Scene01_morning")
		sequenceSettings=("vkitti.yaml")
	fi
	if [ $sequenceGroup == 'Scene02_morning' ]
	then 
		sequenceNames=("Scene02_morning")
		sequenceSettings=("vkitti.yaml")
	fi
	if [ $sequenceGroup == 'Scene06_morning' ]
	then 
		sequenceNames=("Scene06_morning")
		sequenceSettings=("vkitti.yaml")
	fi
	if [ $sequenceGroup == 'Scene18_morning' ]
	then 
		sequenceNames=("Scene18_morning")
		sequenceSettings=("vkitti.yaml")
	fi
	if [ $sequenceGroup == 'Scene20_morning' ]
	then 
		sequenceNames=("Scene20_morning")
		sequenceSettings=("vkitti.yaml")
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
	
	if [ $sequenceGroup == 'miniTestGroup' ]
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
			"corridors_dolly_left"
			"units_dolly_left"
			
		)
		sequenceSettings=( 
			"vectorLeft.yaml"
			"vectorLeft.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'corridors_dolly_left' ]
	then 
		sequenceNames=(
			"corridors_dolly_left"
		)
		sequenceSettings=( 
			"vectorLeft.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'units_dolly_left' ]
	then 
		sequenceNames=(
			"units_dolly_left"
		)
		sequenceSettings=( 
			"vectorLeft.yaml"
		)
	fi
	
fi

if [ $dataset == 'fourseasons' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"office_loop_1_train"
			"office_loop_5_train"
			
		)
		sequenceSettings=( 
			"fourseasons.yaml"
			"fourseasons.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'office_loop_1_train' ]
	then 
		sequenceNames=(
			"office_loop_1_train"
		)
		sequenceSettings=( 
			"fourseasons.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'office_loop_5_train' ]
	then 
		sequenceNames=(
			"office_loop_5_train"
		)
		sequenceSettings=( 
			"fourseasons.yaml"
		)
	fi
	
fi

if [ $dataset == 'eth' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"cables_1"
			"einstein_1"
			"large_loop_1"
			"plant_scene_2"
			"repetitive"
			"table_3"
			
		)
		sequenceSettings=( 
			"eth.yaml" "eth.yaml" "eth.yaml"
			"eth.yaml" "eth.yaml" "eth.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'testGroup' ]
	then 
		sequenceNames=(
			"large_loop_1"
			"repetitive"
			"table_3"
			
		)
		sequenceSettings=( 			
			"eth.yaml" "eth.yaml" "eth.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'miniTestGroup' ]
	then 
		sequenceNames=(
			"large_loop_1"
			"repetitive"
			"table_3"
			
		)
		sequenceSettings=( 			
			"eth.yaml" "eth.yaml" "eth.yaml"
		)
	fi
	
	if [ $sequenceGroup == 'cables_1' ]
	then 
		sequenceNames=(
			"cables_1"
		)
		sequenceSettings=( 
			"eth.yaml"
		)
	fi	
	if [ $sequenceGroup == 'einstein_1' ]
	then 
		sequenceNames=(
			"einstein_1"
		)
		sequenceSettings=( 
			"eth.yaml"
		)
	fi	
	if [ $sequenceGroup == 'large_loop_1' ]
	then 
		sequenceNames=(
			"large_loop_1"
		)
		sequenceSettings=( 
			"eth.yaml"
		)
	fi	
	if [ $sequenceGroup == 'plant_scene_2' ]
	then 
		sequenceNames=(
			"plant_scene_2"
		)
		sequenceSettings=( 
			"eth.yaml"
		)
	fi	
	if [ $sequenceGroup == 'repetitive' ]
	then 
		sequenceNames=(
			"repetitive"
		)
		sequenceSettings=( 
			"eth.yaml"
		)
	fi	
	if [ $sequenceGroup == 'table_3' ]
	then 
		sequenceNames=(
			"table_3"
		)
		sequenceSettings=( 
			"eth.yaml"
		)
	fi		
fi

if [ $dataset == 'interior' ]
then 
	if [ $sequenceGroup == 'all' ]
	then 
		sequenceNames=(
			"3FO4KI08T3TL"
			
		)
		sequenceSettings=( 
			"interior.yaml"
		)
	fi
	
	if [ $sequenceGroup == '3FO4KI08T3TL' ]
	then 
		sequenceNames=(
			"interior-03-24_17-45-31"
		)
		sequenceSettings=( 
			"interior.yaml"
		)
	fi
	
fi
