mkdir train test
cp -r /home/szproxy/Haar/pic_db/INRIAPerson/96X160H96/Train/pos  train/pos
cp -r /home/szproxy/Haar/pic_db/INRIAPerson/Train/neg  train/neg
cp -r /home/szproxy/Haar/pic_db/INRIAPerson/Test/neg  test/neg
#cp -r /home/szproxy/Haar/pic_db/INRIAPerson/70X134H96/Test/pos  test/pos
cp -r /home/szproxy/Haar/pic_db/INRIAPerson/96X160H96/Train/pos  test/pos


find train/neg -type f -print0| xargs -0 -I {} echo $PWD/{} >train/neg.lst
find train/pos  -type f -print0| xargs -0 -I {} echo $PWD/{} > train/pos.lst
find test/neg -type f -print0| xargs -0 -I {} echo $PWD/{} >test/neg.lst
find test/pos  -type f -print0| xargs -0 -I {} echo $PWD/{} > test/pos.lst


./cvCrop train/neg.lst 
#./cvResizeImage train/pos.lst 
./cvCrop train/pos.lst 
./cvCrop test/neg.lst 
#./cvResizeImage test/pos.lst 
./cvCrop test/pos.lst 


find . -name "*.jpg" -o -name "*.png"  | grep -v 64x128 | xargs rm

find train/neg -type f -print0| xargs -0 -I {} echo $PWD/{} >train/neg.lst
find train/pos  -type f -print0| xargs -0 -I {} echo $PWD/{} > train/pos.lst
find test/neg -type f -print0| xargs -0 -I {} echo $PWD/{} >test/neg.lst
find test/pos  -type f -print0| xargs -0 -I {} echo $PWD/{} > test/pos.lst


./hogComputing .
cat Featurestrainpos.txt >> FeatureTrain.txt
cat Featurestrainneg.txt >> FeatureTrain.txt
cat Featurestestpos.txt >> FeatureTest.txt
cat Featurestestneg.txt >> FeatureTest.txt

rm Features*

#svm_light/svm_learn -v 1 -z r -t 1 -i 1 -c 0.01 FeatureTrain.txt svmLightModel.txt
svm_light/svm_learn -i 1 FeatureTrain.txt svmLightModel.txt
#octave makefinalvector.m

#head -11 svmLightModel.txt | tail -1 | sed 's/[\#\,\(\)\*]//g' | sed 's/[a-zA-Z]//g' >> descriptor_out.dat

#tr '\n' '\,' < descriptor_out.dat > descriptor_out2.dat
#sed 's/\,/f,/g' descriptor_out2.dat|sed 's/ *//g' > descriptor_out.dat
#echo >> descriptor_out.dat

rm descriptor_out2.dat
svm_light/svm_classify FeatureTest.txt svmLightModel.txt FeaturePredict.txt
