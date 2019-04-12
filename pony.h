char pony_add_plugin(void(*newplugin)(void));
char pony_init(char*);
char pony_step(void);
char pony_terminate(void);

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

typedef struct                 //инерциальные данные
{
	char* conf;                //конфигурация
	int conflength;            //длина строки конфигурации

	pony_dataArray w;          //гироскопы
	pony_dataArray f;          //акселерометры
	pony_dataArray q;          //кватернион
} pony_imu;

typedef struct                 //
{
	char* conf;                //конфигурация
	int conflength;            //длина строки конфигурации

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
