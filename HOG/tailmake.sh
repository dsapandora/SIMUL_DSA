octave makefinalvector.m

head -11 svmLightModel.txt | tail -1 | sed 's/[\#\,\(\)\*]//g' | sed 's/[a-zA-Z]//g' >> *.dat

tr '\n' '\,' < descriptor_out.dat > descriptor_out2.dat
sed 's/\,/f,/g' descriptor_out2.dat|sed 's/ *//g' > descriptor_out.dat
echo >> descriptor_out.dat

rm descriptor_out2.dat

