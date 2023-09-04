
%% Virtual Kitti Dataset
vkitti.sequences = ["Scene01","Scene02","Scene06","Scene18","Scene20"];
vkitti.subSequences = ["15-deg-left","15-deg-right","30-deg-left","30-deg-right","clone","fog","morning","overcast","rain","sunset"];
vkitti.cameras = ["Camera_0","Camera_1"];

vkitti.numSequences = numel(vkitti.sequences);
vkitti.numSubSequences = numel(vkitti.subSequences);

%% Virtual Kitti Dataset
fourseasons.sequences = ["office_loop_1_train" "office_loop_5_train"];
fourseasons.cameras = ["cam0","cam1"];

fourseasons.numSequences = numel(fourseasons.sequences);
