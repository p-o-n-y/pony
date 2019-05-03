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



typedef struct                 //äëÿ èçìåðåíèé, âûðàæàþùèõñÿ ÷åðåç îäíî ÷èñëî
{
	double val;                //çíà÷åíèå
	unsigned char count;       //ñ÷¸ò÷èê
	unsigned char valid;       //ïðèçíàê âàëèäíîñòè
} pony_data;

typedef struct                 //äëÿ èçìåðåíèé, âûðàæàþùèõñÿ ÷åðåç íåñêîëüêî ÷èñåë
{
	double *val;               //ìàññèâ çíà÷åíèé
	int count;                 //ñ÷¸ò÷èê
	char valid;                //ïðèçíàê âàëèäíîñòè
	int arrsize;               //ðàçìåð ìàññèâà
} pony_dataArray;


// IMU
typedef struct                 //èíåðöèàëüíûå äàííûå
{
	char* cfg;                //êîíôèãóðàöèÿ
	int cfglength;            //äëèíà ñòðîêè êîíôèãóðàöèè

	pony_dataArray w;          //ãèðîñêîïû
	pony_dataArray f;          //àêñåëåðîìåòðû
	pony_dataArray q;          //êâàòåðíèîí
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

	double *obs;			// ìàññèâ èçìåðåíèé íà òåêóùèé ìîìåíò, îïðåäåëÿåòñÿ at runtime

} pony_gnss_gps_sat;

typedef struct                 //
{
	char* cfg;                //êîíôèãóðàöèÿ
	int cfglength;            //äëèíà ñòðîêè êîíôèãóðàöèè

	pony_gnss_gps_sat *sat;			// ñïóòíèêè
	int max_sat_num;
	int max_eph_count;

	pony_dataArray iono_a;
	pony_dataArray iono_b;
	pony_dataArray clock_corr;
} pony_gnss_gps;

typedef struct                 //
{
	char* cfg;                //êîíôèãóðàöèÿ
	int cfglength;            //äëèíà ñòðîêè êîíôèãóðàöèè

	pony_gnss_gps_sat *sat;			// ñïóòíèêè
	int max_sat_num;
	int max_eph_count;

	pony_dataArray iono_a;
	pony_dataArray iono_b;
	pony_dataArray clock_corr;
} pony_gnss_glo;

typedef struct                 //ñïóòíèêîâûå äàííûå
{
	char* cfg;                //êîíôèãóðàöèÿ
	int cfglength;            //äëèíà ñòðîêè êîíôèãóðàöèè

	pony_gnss_gps* gps;
	pony_gnss_glo* glo;

	pony_time_epoch epoch;

	char* wcfg;
	int wcfglength;

} pony_gnss;


// CORE
#define pony_bus_version 0     //âåðñèÿ øèíû 
typedef struct                 //øèíà
{
	int ver;                   //âåðñèÿ øèíû, äëÿ èñïîëüçîâàíèÿ â runtime
	int mode;                  //ðåæèì ðàáîòû: 0 èíèöèàëèçàöèÿ, >0 ðàáîòà, <0 çàâåðøåíèå

	pony_imu* imu;
	pony_gnss* gnss;

	pony_data t;               //âðåìÿ

	char* cfg;                //êîíôèãóðàöèÿ
	int cfglength;            //äëèíà ñòðîêè êîíôèãóðàöèè
} pony_bus;

typedef struct
{
	pony_bus bus;

	void(**plugins)(void);     //óêàçàòåëü íà ìàññèâ óêàçàòåëåé íà ïëàãèíû
	int pluginsNum;            //êîëè÷åñòâî ïëàãèíîâ

	char* cfg;                //êîíôèãóðàöèÿ
	int cfglength;            //äëèíà ñòðîêè êîíôèãóðàöèè

	int exitplnum;             //íîìåð ïëàãèíà, âûçâàâøåãî çàâåðøèâøåíèå ðàáîòû
} pony_struct;

extern pony_struct pony;
