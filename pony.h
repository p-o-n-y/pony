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



typedef struct                 //для измерений, выражающихся через одно число
{
	double val;                //значение
	unsigned char count;       //счётчик
	unsigned char valid;       //признак валидности
} pony_data;

typedef struct                 //для измерений, выражающихся через несколько чисел
{
	double *val;               //массив значений
	int count;                 //счётчик
	char valid;                //признак валидности
	int arrsize;               //размер массива
} pony_dataArray;


// IMU
typedef struct                 //инерциальные данные
{
	char* conf;                //конфигурация
	int conflength;            //длина строки конфигурации

	pony_dataArray w;          //гироскопы
	pony_dataArray f;          //акселерометры
	pony_dataArray q;          //кватернион
} pony_imu;



// GNSS
typedef struct {

	char prn;

	double x[3];
	double v[3];
	double t_em;

	double eph[32];
	
	char visible;
	double sinEl;

} pony_gps_sat;

typedef struct                 //
{
	char* conf;                //конфигурация
	int conflength;            //длина строки конфигурации

	pony_gps_sat *sat;
	int max_sat_num;
} pony_gnss_gps;

typedef struct                 //
{
	char* conf;                //конфигурация
	int conflength;            //длина строки конфигурации

} pony_gnss_glo;

typedef struct                 //спутниковые данные
{
	char* conf;                //конфигурация
	int conflength;            //длина строки конфигурации

	pony_gnss_gps* gps;
	pony_gnss_glo* glo;

	char* wconf;
	int wconflength;

} pony_gnss;


// CORE
#define pony_bus_version 0     //версия шины 
typedef struct                 //шина
{
	int ver;                   //версия шины, для использования в runtime
	int mode;                  //режим работы: 0 инициализация, >0 работа, <0 завершение

	pony_imu* imu;
	pony_gnss* gnss;

	pony_data t;               //время

	char* conf;                //конфигурация
	int conflength;            //длина строки конфигурации
} pony_bus;

typedef struct
{
	pony_bus bus;

	void(**plugins)(void);     //указатель на массив указателей на плагины
	int pluginsNum;            //количество плагинов

	char* conf;                //конфигурация
	int conflength;            //длина строки конфигурации

	int exitplnum;             //номер плагина, вызвавшего завершившение работы
} pony_struct;

extern pony_struct pony;

