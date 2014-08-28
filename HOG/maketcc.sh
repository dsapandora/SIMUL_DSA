TCCHEADER='const double PERSON_WEIGHT_VEC[] = {'
TCCTAIL='};

const int PERSON_WEIGHT_VEC_LENGTH = sizeof(PERSON_WEIGHT_VEC)/sizeof(double);'
echo "$TCCHEADER"
cat $1
echo "$TCCTAIL"
