/*---------- IO functions used for reading/writting .GPF -----------*/
int OpenPGfileR(char *fileRname);
int ClosePGfileR(void );
char * fgetline(int * length);

int OpenPGfileW(char *filename);     /* opens a PG/GF file for writting */
int ClosePGfileW(void);                  /* closes a the PG/GF file */
char * fputline(char *s); 
char * fput(char * s);  
void SetBreakLength(unsigned int l);


