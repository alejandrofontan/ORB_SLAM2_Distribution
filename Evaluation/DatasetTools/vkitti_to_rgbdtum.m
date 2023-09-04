close all;
clear all;

%% Settings
datasetPath = "/home/alex/VKITTI";
imageType = '*.jpg';
imagesSubFolder = "frames/rgb";
hz = 10;
iCamera = 1;
%% Load Sequence Names
run datasetSequences.m

for iSequence = 1:1:vkitti.numSequences
    for iSubSequence = 1:1:vkitti.numSubSequences

        %%
        sourceFolder = fullfile(datasetPath,vkitti.sequences{iSequence},vkitti.subSequences{iSubSequence});
        destinationFolder = fullfile(datasetPath,vkitti.sequences{iSequence} + "_" + vkitti.subSequences{iSubSequence});
        success = movefile(sourceFolder, destinationFolder, 'f');

        %% Load Images
        sequence = fullfile(vkitti.sequences{iSequence} + "_" + vkitti.subSequences{iSubSequence})
        sequencePath = fullfile(datasetPath,sequence);
        images =  dir(fullfile(sequencePath,imagesSubFolder,vkitti.cameras(iCamera),imageType));
        
        %% Create rgb.txt file
        %   imageTimestamp imagePath
        rgbTxt = [];
        for iImage = 1:1:numel(images)
            timeStamp = (1/hz)*(iImage-1);
        
            fileName = fullfile(imagesSubFolder,vkitti.cameras(iCamera),images(iImage).name);
            rgbTxt = [rgbTxt;[timeStamp, fileName]];
        end
        writematrix(rgbTxt,fullfile(sequencePath,"rgb.txt"),'Delimiter',' ');
        
        %% Create groundtruth.txt file
        %   imageTimestamp tx ty tz qx qy qz qw

        gt_raw = readtable(fullfile(datasetPath,"vkitti_2.0.3_textgt",vkitti.sequences{iSequence},vkitti.subSequences{iSubSequence},"extrinsic.txt"),'ReadVariableNames', true);
        gtTxt = [];
        imageId = 1;
        for rowIdx = 1:size(gt_raw, 1)
            currentRow = gt_raw(rowIdx, :);
            % Process the current row
            if currentRow.cameraID == (iCamera-1)
                %disp(currentRow);
                R(1,1) = currentRow.r1_1;
                R(1,2) = currentRow.r1_2;
                R(1,3) = currentRow.r1_3;
                R(2,1) = currentRow.r2_1;
                R(2,2) = currentRow.r2_2;
                R(2,3) = currentRow.r2_3;
                R(3,1) = currentRow.r3_1;
                R(3,2) = currentRow.r3_2;
                R(3,3) = currentRow.r3_3;
        
                R = R';
                quaternion = rotm2quat(R); % q = [w x y z]
                quaternion = [quaternion(2:4),quaternion(1)];
        
                t(1) = currentRow.t1;
                t(2) = currentRow.t2;
                t(3) = currentRow.t3;
        
                t = -(R*t')';
        
                newLine = double([rgbTxt(imageId),t,quaternion]);
                gtTxt = [gtTxt;newLine];
        
                imageId = imageId + 1;
            end
        end
        writematrix(gtTxt,fullfile(sequencePath,"groundtruth.txt"),'Delimiter',' ');
    end
end
