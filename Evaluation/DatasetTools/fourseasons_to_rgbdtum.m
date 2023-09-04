close all;
clear all;

%% Settings
datasetPath = "/home/alex/FOURSEASONS";
imageType = '*.png';
imagesSubFolder = "undistorted_images";
hz = "?";
iCamera = 1;
%% Load Sequence Names
run datasetSequences.m

for iSequence = 1:1:fourseasons.numSequences
    %% Load Images
    sequence = fourseasons.sequences{iSequence}
    sequencePath = fullfile(datasetPath,sequence);
    images =  dir(fullfile(sequencePath,imagesSubFolder,fourseasons.cameras(iCamera),imageType));
        
    %% Create rgb.txt file
    %   imageTimestamp imagePath
    rgbTxt = [];
    for iImage = 1:1:numel(images)
        timeStamp =  double(string(images(iImage).name(1:end-4)))/1e9;
        fileName = fullfile(imagesSubFolder,vkitti.cameras(iCamera),images(iImage).name);
        rgbTxt = [rgbTxt;[timeStamp, fileName]];
    end
    writematrix(rgbTxt,fullfile(sequencePath,"rgb.txt"),'Delimiter',' ');
        
    %% Create groundtruth.txt file
    %   imageTimestamp tx ty tz qx qy qz qw

    gt_raw = readtable(fullfile(sequencePath,"GNSSPoses.txt"),'Delimiter',',');
    gtTxt = [];
    imageId = 1;
    for rowIdx = 1:size(gt_raw, 1)
        currentRow = gt_raw(rowIdx, :);
            
        quaternion = [currentRow.Var5,currentRow.Var6,currentRow.Var7,currentRow.Var8];
        
        t(1) = currentRow.Var2;
        t(2) = currentRow.Var3;
        t(3) = currentRow.Var4;
        
        newLine = double([rgbTxt(imageId),t,quaternion]);
        gtTxt = [gtTxt;newLine];
        
        imageId = imageId + 1;      
    end
    writematrix(gtTxt,fullfile(sequencePath,"groundtruth.txt"),'Delimiter',' ');
end