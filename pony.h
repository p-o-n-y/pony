char pony_add_plugin(void(*newplugin)(void));
char pony_init(char*);
char pony_step(void);
char pony_terminate(void);



char pony_extract_string_length(char* confstr, int length, char* identifier, int* res);
char pony_extract_string(char* confstr, int length, char* identifier, char** res);
char pony_extract_char_sym(char* confstr, int length, char* identifier, char* res);
char pony_extract_char_num(char* confstr, int length, char* identifier, char* res);
char pony_extract_short(char* confstr, int length, char* identifier, short* res);
char pony_extract_int(char* confstr, int length, char* identifier, int* res);
char pony_extract_long(char* confstr, int length, char* identifier, long* res);
char pony_extract_float(char* confstr, int length, char* identifier, float* res);
char pony_extract_double(char* confstr, int length, char* identifier, double* res);
char pony_extract_bool(char* confstr, int length, char* identifier, char* res);



typedef struct                 
{
	double val;                
	unsigned char count;       
	unsigned char valid;      
} pony_data;

typedef struct                 
{
	double *val;               
	int count;                 
	char valid;                
	int arrsize;               
} pony_dataArray;



// CORE
#define pony_bus_version 0      
typedef struct                 
{
	int ver;                   
	int mode;                  

	pony_imu* imu;
	pony_gnss* gnss;

	pony_data t;               

	char* cfg;                
	int cfglength;            
} pony_bus;

typedef struct
{
	pony_bus bus;

	void(**plugins)(void);     
	int pluginsNum;            

	char* cfg;                
	int cfglength;            

	int exitplnum;             
} pony_struct;

extern pony_struct pony;



// IMU
typedef struct                 
{
	char* cfg;                
	int cfglength;            

	pony_dataArray w;          
	pony_dataArray f;          
	pony_dataArray q;          
	pony_data dt;
} pony_imu;



// GNSS
typedef struct {
	int Y;
	int M;
	int D;
	int h;
	int m;
	double s;
} pony_time_epoch;

typedef struct {

	double *eph;

	double t_em;
	double x[3];
	double v[3];

	char visible;
	double sinEl;

	double *obs;			

} pony_gnss_sat;

typedef struct                 
{
	char* cfg;                
	int cfglength;            

	pony_gnss_sat *sat;			
	int max_sat_num;
	int max_eph_count;

	pony_dataArray iono_a;
	pony_dataArray iono_b;
	pony_dataArray clock_corr;
} pony_gnss_gps;

typedef struct                
{
	char* cfg;                
	int cfglength;            

	pony_gnss_sat *sat;			
	int max_sat_num;
	int max_eph_count;

	pony_dataArray iono_a;
	pony_dataArray iono_b;
	pony_dataArray clock_corr;
} pony_gnss_glo;

typedef struct                
{
	char* cfg;                
	int cfglength;            

	pony_gnss_gps* gps;
	pony_gnss_glo* glo;

	pony_time_epoch epoch;

	char* wcfg;
	int wcfglength;

} pony_gnss;
