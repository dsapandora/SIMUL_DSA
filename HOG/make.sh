mkdir train test
cp -r /home/szproxy/Haar/pic_db/INRIAPerson/96X160H96/Train/pos  train/pos
cp -r /home/szproxy/Haar/pic_db/INRIAPerson/Train/neg  train/neg
cp -r /home/szproxy/Haar/pic_db/INRIAPerson/Test/neg  test/neg
#cp -r /home/szproxy/Haar/pic_db/INRIAPerson/70X134H96/Test/pos  test/pos
#cp -r /home/szproxy/Haar/pic_db/INRIAPerson/96X160H96/Train/pos  test/pos


find train/neg -type f -print0| xargs -0 -I {} echo $PWD/{} >train/neg.lst
find train/pos  -type f -print0| xargs -0 -I {} echo $PWD/{} > train/pos.lst
find test/neg -type f -print0| xargs -0 -I {} echo $PWD/{} >test/neg.lst
#find test/pos  -type f -print0| xargs -0 -I {} echo $PWD/{} > test/pos.lst


./cvCrop -r train/neg.lst 
./cvCrop train/pos.lst 
./cvCrop -r test/neg.lst 
#./cvCrop test/pos.lst 


find . -name "*.jpg" -o -name "*.png"  | grep -v 64x128 | xargs rm

find train/neg -type f -print0| xargs -0 -I {} echo $PWD/{} >train/neg.lst
find train/pos  -type f -print0| xargs -0 -I {} echo $PWD/{} > train/pos.lst
find test/neg -type f -print0| xargs -0 -I {} echo $PWD/{} >test/neg.lst
#find test/pos  -type f -print0| xargs -0 -I {} echo $PWD/{} > test/pos.lst


./cvHogFeatureCal -n train .
./cvHogFeatureCal -p train .
./cvHogFeatureCal -n test .

cat Featurestrainpos.txt > FeatureTrain.txt
cat Featurestrainneg.txt >> FeatureTrain.txt
#cat Featurestestpos.txt >> FeatureTest.txt
cat Featurestestneg.txt >> FeatureTest.txt

#rm Features*

svm_light/svm_learn -v 1 -t 0 -c 0.01 FeatureTrain.txt svmLightModel.txt
# differ than hogCompute3.
svm_light/svm_classify FeatureTest.txt svmLightModel.txt FeaturePredict.txt

paste FeaturePredict.txt test/neg.lst > test/score.lst 
mkdir train/hardneg
grep -v - test/score.lst | awk '{print $2}' | xargs -I {} cp {} train/hardneg
cp train/neg/* train/hardneg
find train/hardneg -type f -print0| xargs -0 -I {} echo $PWD/{} >train/hardneg.lst
./cvHogFeatureCal -h train .
cat Featurestrainpos.txt > FeatureHardTrain.txt
cat Featurestrainhardneg.txt >> FeatureHardTrain.txt
svm_light/svm_learn -v 1 -t 0 -c 0.01 FeatureHardTrain.txt svmLightHardModel.txt
svm_light/svm_classify FeatureHardTrain.txt svmLightHardModel.txt FeatureHardPredict.txt
paste FeatureHardPredict.txt train/hardneg.lst > train/hardscore.lst 




./weightVector.pl svmLightHardModel.txt | awk -F: '{print $2}' > descriptor_hard.dat
tr '\n' '\,' < descriptor_hard.dat > descriptor_hard_tcc.dat

head -11 svmLightModel.txt | tail -1 | sed 's/[\#\,\(\)\*]//g' | sed 's/[a-zA-Z]//g' >> descriptor_hard.dat

tr '\n' '\,' < descriptor_hard.dat > descriptor_hard2.dat
sed 's/\,/f,/g' descriptor_hard2.dat|sed 's/ *//g' > descriptor_hard.dat
echo >> descriptor_hard.dat

#rm descriptor_out2.dat
#svm_light/svm_classify FeatureTest.txt svmLightModel.txt FeaturePredict.txt
./makepeople.sh descriptor_hard.dat > /home/szproxy/save/cvSamplePeopleDetect/peopledetect.cpp
./maketcc.sh descriptor_hard_tcc.dat > /home/szproxy/__Feature__/workingOLT/lib/persondetectorwt.tcc
