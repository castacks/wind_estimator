clear();
delete(findall(0,'Type','figure'));
   
%==========================================================================
%===========================================================
%% Import wind_estimator data
%==========================================================================
%===========================================================
[time cekf swkf maf] = loadWindEstimatorData('2018-01-25/Flight2/wind_estimator_2018_01_25_12_22_03.dat');

cekf_Headings = getHeadingsFromVector(cekf(:,1),cekf(:,2));
cekf_Magnitude = zeros(size(time,1),1);
swkf_Headings = getHeadingsFromVector(swkf(:,1),swkf(:,2));
swkf_Magnitude = zeros(size(time,1),1);
maf_Headings = getHeadingsFromVector(maf(:,1),maf(:,2));
maf_Magnitude = zeros(size(time,1),1);

for i = 1:size(time,1)
    cekf_Magnitude(i) = norm(transpose(cekf(i,1:3)));
    swkf_Magnitude(i) = norm(transpose(swkf(i,1:3)));
    maf_Magnitude(i) = norm(transpose(maf(i,1:3)));
end

%%
figure();
subplot(2,1,1);
plot(time, cekf_Magnitude, time, swkf_Magnitude, time, maf_Magnitude);
legend('CEKF','SWKF','MAF');

subplot(2,1,2);
plot(time, cekf_Headings, time, swkf_Headings, time, maf_Headings);
legend('CEKF','SWKF','MAF');
