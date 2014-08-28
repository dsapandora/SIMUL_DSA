/** 
 * @author Navneet Dalal (Navneet.Dalal@inrialpes.fr)
 * Support for binary data format added to SVM Light, as it takes too 
 * long to read supported text format.
 */
#include "svm_common.h"
#include <stdio.h>

void typeid_verbose(int typeid) {
      switch (typeid) {
          case 3:
              printf("Will read 'int' type target values\n");
              break;
          case 4:
              printf("Will read 'float' type target values\n");
              break;
          case 5:
              printf("Will read 'double' type target values\n");
              break;
          default:
              printf("Default type-id. Will read double type "
                      "target values\n");
              break;
      };
}
void read_binary_documents(char *docfile, DOC ***docs, double **label, 
		    long int *totwords, long int *totdoc)
{// {{{
  char *comment;
  WORD *words;
  long dnum=0,wpos,dpos=0,dneg=0,dunlab=0,queryid,slackid,max_docs;
  long max_words_doc;
  double costfactor;
  double doc_label;
  FILE *docfl;

  if ((docfl = fopen (docfile, "rb")) == NULL)
  { perror (docfile); exit (1); }

  /* read version number */
  int version = 0;
  if (!fread (&version,sizeof(int),1,docfl))
  { perror ("Unable to read version number"); exit (1); }

  int data_typeid = 0, target_typeid = 0;
  if (!fread (&data_typeid,sizeof(int),1,docfl))
  { perror ("Unable to read data type id"); exit (1); }
  if (!fread (&target_typeid,sizeof(int),1,docfl))
  { perror ("Unable to read target type id"); exit (1); }

  if(verbosity>=1) {
      typeid_verbose(data_typeid);
      typeid_verbose(target_typeid);
  }

  /* scan size of input file */
  int feature_length = 0, num_feature = 0;
  if (!fread (&num_feature,sizeof(int),1,docfl))
  { perror ("Unable to read number of feature"); exit (1); }

  if (!fread (&feature_length,sizeof(int),1,docfl))
  { perror ("Unable to read feature vector length"); exit (1); }

  max_words_doc = feature_length; max_docs = num_feature;
  (*totwords)=max_words_doc;
  if(verbosity>=1) {
    printf("Feature length %d, Feature count %d\n",
            feature_length,num_feature);
  }

  if((*totwords) > MAXFEATNUM) {
    printf("\nMaximum feature number exceeds limit defined in MAXFEATNUM!\n");
    exit(1);
  }
  /* set comment to something for time being */
  comment = (char*) my_malloc(sizeof(char)*1);
  *comment = 0;

  (*docs) = (DOC **)my_malloc(sizeof(DOC *)*max_docs);    /* feature vectors */
  (*label) = (double *)my_malloc(sizeof(double)*max_docs); /* target values */

  words = (WORD *)my_malloc(sizeof(WORD)*(max_words_doc+1));
  dnum=0;

  if(verbosity>=2) {
    printf("Reading examples into memory..."); fflush(stdout);
  }
  while(!feof(docfl) && dnum < max_docs) {
    /* wpos contains type id for time being*/
    if(!read_feature(docfl,words,&doc_label,
                target_typeid, data_typeid,
                &queryid,&slackid,&costfactor,
		&wpos,max_words_doc,&comment)) {
      printf("\nParsing error in vector %ld!\n",dnum);
      exit(1);
    }
    (*label)[dnum]=doc_label;
    /*printf("docnum=%ld: Class=%d ",dnum,doc_label); fflush(stdout);*/
    if (doc_label < 0) dneg++;
    else if(doc_label > 0) dpos++;
    else if (doc_label == 0) dunlab++;
    (*docs)[dnum] = create_example(dnum,queryid,slackid,costfactor,
                        create_svector(words,*totwords,comment,1.0));
    /*printf("\nNorm=%f\n",((*docs)[dnum]->fvec)->twonorm_sq);*/ 
    dnum++;  
    if(verbosity>=2 && ((dnum % 100) == 0)) {
	printf("%ld..",dnum); fflush(stdout);
    }
  } 

  fclose(docfl);
  free(words);
  if(verbosity>=1) {
    fprintf(stdout, "OK. (%ld examples read)\n", dnum);
  }
  (*totdoc)=dnum;
}// }}}
int read_feature(FILE *docfl, WORD *words, double *label,
        int target_typeid, int data_typeid,
        long *queryid, long *slackid, double *costfactor,
        long int *numwords, long int max_words_doc,
        char **comment)
{// {{{
  register long wpos;

  (*queryid)=0;
  (*slackid)=0;
  (*costfactor)=1;
  /* do not modify comment for time being*/
  /* (*comment)=NULL; */

  if (feof(docfl) == EOF) {
      perror("premature EOF");
      return 0;
  } else if (ferror(docfl)) {
      perror("Unexpected error, unable to read all features");
      return 0;
  }

    /* read the target value */
    switch (target_typeid) {
        double dlabel; /* store label in case typeid is double*/
        float flabel; /* store label in case typeid is float*/
        int ilabel; /* store label in case typeid is int*/

        case 3:
        if (fread(&ilabel, sizeof(int),1,docfl) <1) {
            perror("Unable to read label");
            return 0;
        }
        *label = ilabel;
        break;
        case 4:
        if (fread(&flabel, sizeof(float),1,docfl) <1) {
            perror("Unable to read label");
            return 0;
        }
        *label = flabel;
        break;
        case 5:  default:
        if (fread(&dlabel, sizeof(double),1,docfl) <1) {
            perror("Unable to read label");
            return 0;
        }
        *label = dlabel;
        break;
    };
    switch (data_typeid) {
        case 3:
            for (wpos=0; wpos<max_words_doc; ++wpos) 
            {   
                int iweight;
                if (fread(&iweight, sizeof(int),1,docfl) <1) {
                    perror("Unable to read feature vector element");
                    return 0;
                }
                (words[wpos]).wnum=wpos+1;
                (words[wpos]).weight=(FVAL)iweight;
            }
            break;
        case 4:
            for (wpos=0; wpos<max_words_doc; ++wpos) 
            {   
                float fweight;
                if (fread(&fweight, sizeof(float),1,docfl) <1) {
                    perror("Unable to read feature vector element");
                    return 0;
                }
                (words[wpos]).wnum=wpos+1;
                (words[wpos]).weight=(FVAL)fweight;
            }
            break;
        case 5:  default:
            for (wpos=0; wpos<max_words_doc; ++wpos) 
            {   
                double dweight;
                if (fread(&dweight, sizeof(double),1,docfl) <1) {
                    perror("Unable to read feature vector element");
                    return 0;
                }
                (words[wpos]).wnum=wpos+1;
                (words[wpos]).weight=(FVAL)dweight;
            }
            break;
    };
  (words[wpos]).wnum=0;
  (*numwords)=wpos+1;
  return(1);
}// }}}

void write_binary_model(const char *modelfile, MODEL *model)
{// {{{
  FILE *modelfl;
  long j,i,sv_num;
  SVECTOR *v;

  if(verbosity>=3) {
    printf("Writing model file..."); fflush(stdout);
  }
  if ((modelfl = fopen (modelfile, "wb")) == NULL)
  { perror (modelfile); exit (1); 
  }
  char version_buffer[10];
  for (i= 0; i< 10; ++i) 
      version_buffer[i] = 0;
  int version_len = strlen(VERSION);
  if (version_len >10)
    version_len = 10;
  strncpy(version_buffer,VERSION,version_len);

  fwrite(version_buffer,sizeof(char),10,modelfl);
//    fprintf(modelfl,"SVM-light Version %s\n",VERSION);
  int version;
#if DENSE
  version = 200;
#else
  version = 100;
#endif
  fwrite(&version,sizeof(int),1,modelfl);
  fwrite(&(model->kernel_parm.kernel_type),sizeof(long),1,modelfl);
  fwrite(&(model->kernel_parm.poly_degree),sizeof(long),1,modelfl);
  fwrite(&(model->kernel_parm.rbf_gamma),sizeof(double),1,modelfl);
  fwrite(&(model->kernel_parm.coef_lin),sizeof(double),1,modelfl); 
  fwrite(&(model->kernel_parm.coef_const),sizeof(double),1,modelfl);
  long l = strlen(model->kernel_parm.custom);
  fwrite(&l,sizeof(long),1,modelfl);
  fwrite(model->kernel_parm.custom,sizeof(char),l,modelfl);
  fwrite(&(model->totwords),sizeof(long),1,modelfl);
  fwrite(&(model->totdoc),sizeof(long),1,modelfl);
 
  sv_num=1;
  for(i=1;i<model->sv_num;++i) {
    for(v=model->supvec[i]->fvec;v;v=v->next) 
      sv_num++;
  }
  fwrite(&sv_num, sizeof(long),1,modelfl);
  fwrite(&(model->b), sizeof(double),1,modelfl);

  if(model->kernel_parm.kernel_type == 0) { /* linear kernel */
    add_weight_vector_to_linear_model(model);

    /* save linear wts */
    fwrite(model->lin_weights, sizeof(double),model->totwords+1,modelfl);
  } else {
    for(i=1;i<model->sv_num;++i) {
        for(v=model->supvec[i]->fvec;v;v=v->next) {
        double wt = model->alpha[i]*v->factor;
        fwrite(&wt, sizeof(double),1,modelfl);
#if DENSE
        for (j=0; j < v->n_words; ++j) {
                fwrite(&(v->words[j]), sizeof(double),1,modelfl);
        }
#else
        for (j=0; (v->words[j]).wnum; ++j) {
                fwrite(&((v->words[j]).wnum), sizeof(long),1,modelfl);
                fwrite(&((v->words[j]).weight), sizeof(double),1,modelfl);
        }
        fwrite(&((v->words[j]).wnum), sizeof(long),1,modelfl);
#endif
        long ll = strlen(v->userdefined)+1;
        fwrite(&ll,sizeof(long),1,modelfl);
        fwrite(v->userdefined,sizeof(char),ll,modelfl);
        /* NOTE: this could be made more efficient by summing the
        alpha's of identical vectors before writing them to the
        file. */
        }
    }
  }
  fclose(modelfl);
  if(verbosity>=3) {
    printf("done\n");
  }
}// }}}

MODEL *read_binary_model(const char *modelfile)
{// {{{
  FILE *modelfl;
  long i,j;
  SVECTOR *v;

  MODEL *model= (MODEL *)my_malloc(sizeof(MODEL));

  if(verbosity>=2) {
    printf("Reading model..."); fflush(stdout);
  }
  if ((modelfl = fopen (modelfile, "rb")) == NULL)
  { perror (modelfile); exit (1); }

  char version_buffer[10];
  if (!fread (&version_buffer,sizeof(char),10,modelfl))
  { perror ("Unable to read version"); exit (1); }
  if(strcmp(version_buffer,VERSION)) {
    perror ("Version of model-file does not match version of svm_classify!"); 
    exit (1); 
  }
  /* read version number */
  int version = 0;
  if (!fread (&version,sizeof(int),1,modelfl))
  { perror ("Unable to read version number"); exit (1); }
#if DENSE
  if (version != 200)
  { perror("Model file compiled for light version"); exit(1);}
#else
  if (version != 100)
  { perror("Model file compiled for dense version"); exit(1);}
#endif
//    int data_typeid = 0, target_typeid = 0;
//    if (!fread (&data_typeid,sizeof(int),1,modelfl))
//    { perror ("Unable to read data type id"); exit (1); }
//    if (!fread (&target_typeid,sizeof(int),1,modelfl))
//    { perror ("Unable to read target type id"); exit (1); }
//    if(verbosity>=1) {
//        typeid_verbose(data_typeid);
//        typeid_verbose(target_typeid);
//    }

  KERNEL_PARM* kp= &(model->kernel_parm);
  fread(&(kp->kernel_type),sizeof(long),1,modelfl);
  fread(&(kp->poly_degree),sizeof(long),1,modelfl);
  fread(&(kp->rbf_gamma),sizeof(double),1,modelfl);
  fread(&(kp->coef_lin),sizeof(double),1,modelfl); 
  fread(&(kp->coef_const),sizeof(double),1,modelfl);
  long l;
  fread(&l,sizeof(long),1,modelfl);
  fread(model->kernel_parm.custom,sizeof(char),l,modelfl);
  fread(&(model->totwords),sizeof(long),1,modelfl);
  fread(&(model->totdoc),sizeof(long),1,modelfl);
  fread(&(model->sv_num), sizeof(long),1,modelfl);
  fread(&(model->b), sizeof(double),1,modelfl);

  if(model->kernel_parm.kernel_type == 0) { /* linear kernel */
    /* save linear wts also */
    model->lin_weights =(double *)my_malloc(sizeof(double)*(model->totwords+1));
    fread(model->lin_weights, sizeof(double),model->totwords+1,modelfl);

    model->supvec = NULL;
    model->alpha = NULL;
    model->index = NULL;
  } else {
#if DENSE
    FVAL *words= (FVAL *)my_malloc(sizeof(FVAL)*(model->totwords+1));
#else
    WORD *words = (WORD *)my_malloc(sizeof(WORD)*(model->totwords+1));
#endif

    model->supvec = (DOC **)my_malloc(sizeof(DOC *)*model->sv_num);
    model->alpha = (double *)my_malloc(sizeof(double)*model->sv_num);
    model->index=NULL;
    if(model->kernel_parm.kernel_type != 0) { /* linear kernel */
        model->lin_weights=NULL;
    }

    for(i=1;i<model->sv_num;++i) {
        fread(&(model->alpha[i]), sizeof(double),1,modelfl);
#if DENSE
        for (j=0; j < model->totwords; ++j) {
                fread(&(words[j]), sizeof(double),1,modelfl);
        }
#else
        fread(&((words[0]).wnum), sizeof(long),1,modelfl);
        for (j=0; (words[j]).wnum; ++j) {
                fread(&((words[j]).weight), sizeof(double),1,modelfl);
                fread(&((words[j+1]).wnum), sizeof(long),1,modelfl);
        }
#endif
        long ll;
        fread(&ll,sizeof(long),1,modelfl);
        char* comment = (char *)my_malloc(sizeof(char)*ll);
        fread(comment,sizeof(char),ll,modelfl);
#if DENSE
        model->supvec[i] = create_example(-1, 0,0, 0.0,
            create_ns_svector(words,model->totwords,comment,1.0));
#else
        model->supvec[i] = create_example(-1, 0,0, 0.0,
            create_svector(words,model->totwords,comment,1.0));
#endif
    }
    free(words);
  }

  fclose(modelfl);
  if(verbosity>=2) {
    fprintf(stdout, "OK. (%d support vectors read)\n",(int)(model->sv_num-1));
  }
  return(model);
}// }}}
