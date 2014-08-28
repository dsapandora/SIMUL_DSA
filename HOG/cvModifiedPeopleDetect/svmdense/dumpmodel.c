# include "svm_common.h"

char modelfile[200];
char outfile[200];

void read_input_parameters(int, char **, char *, char *, long *, long*);
void print_help(void);

void write_binary_model(const char *modelfile, MODEL *model);

int main (int argc, char* argv[])
{
  MODEL *model; 

  read_input_parameters(argc,argv,modelfile,outfile, &verbosity, &format);

  if (format) {
    model=read_binary_model(modelfile);
  } else {
    model=read_model(modelfile);
    if(model->kernel_parm.kernel_type == 0) { /* linear kernel */
        /* compute weight vector */
        add_weight_vector_to_linear_model(model);
    }
  }
    if(model->kernel_parm.kernel_type == 0) { /* linear kernel */
        FILE* modelfl = fopen (outfile, "wb");
        if (modelfl==NULL)
        { perror (modelfile); exit (1); }

        if (verbosity > 1)
            fprintf(modelfl,"B=%.32g\n",model->b);
        long i=0;
        for (i= 0; i< model->totwords; ++i) 
            fprintf(modelfl,"%.32g\n",model->lin_weights[i]);
    } else {
        fprintf(stderr,"No output besides linear models\n");
    }
  free_model(model,1);
  return(0);
}

void read_input_parameters(int argc, char **argv, 
        char *modelfile, char *outfile, long int *verbosity, long int *format)
{
  long i;
  
  /* set default */
  strcpy (modelfile, "svm_model");
  strcpy (outfile, "mode.blt"); 
  (*verbosity)=1;
  (*format)=1;

  for(i=1;(i<argc) && ((argv[i])[0] == '-');i++) {
    switch ((argv[i])[1]) 
      { 
      case 'h': print_help(); exit(0);
      case 'v': i++; (*verbosity)=atol(argv[i]); break;
      case 'B': i++; (*format)= atol(argv[i]); break;
      default: printf("\nUnrecognized option %s!\n\n",argv[i]);
	       print_help();
	       exit(0);
      }
  }
  if(argc - i - 2 < 0) {
    printf("\nNot enough input parameters!\n\n");
    print_help();
    exit(0);
  }
  strcpy (modelfile, argv[i]);
  strcpy (outfile, argv[i+1]);
}

void print_help(void)
{
  printf("\nSVM-light %s: Support Vector Machine, convert model to binary file %s\n",VERSION,VERSION_DATE);
  copyright_notice();
  printf("   usage: svm_classify [options] model_file output_file\n\n");
  printf("options: -h         -> this help\n");
  printf("         -v [0..3]  -> verbosity level (default 1)\n");
  printf("         -B [0,1]    -> binary input files (default 1)\n");
}




