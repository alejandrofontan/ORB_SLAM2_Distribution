# Create Vocabulary
feature="brief" # orb, akaze, surf, kaze, sift, brief

rm -rf "/home/alex/ORB_SLAM2_Distribution/Vocabulary/${feature}_DBoW2_voc.yml"
./../../Thirdparty/DBoW2/build/demo ${feature} "/home/alex/ORB_SLAM2_Distribution/Vocabulary" "/home/alex/ORB_SLAM2_Distribution/Evaluation/Templates/BovisaImages.txt"
gzip -d "/home/alex/ORB_SLAM2_Distribution/Vocabulary/${feature}_DBoW2_voc.yml.gz"
