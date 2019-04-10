void pony_add_plugin(void*newplugin(void));
void pony_init(char*);
char pony_step();
void pony_terminate();

typedef struct                 //для измерений, выражающихся через одно число
{
	double val;                //измерение
	int count;                 //счётчик
	char valid;                //признак валидности
} pony_data;

typedef struct                 //для измерений, выражающихся через несколько чисел
{
	double *val;               //массив измерений
	int count;                 //счётчик
	char valid;                //признак валидности
	unsigned char arrsize;     //размер массива
} pony_dataArray;

typedef struct                 //инерциальные данные
{
	char* conf;                //конфигурация
	char conflength;           //длина строки конфигурации

	pony_dataArray w;          //гироскопы
	pony_dataArray f;          //акселерометры
	pony_dataArray q;          //кватернион
} pony_imu;

typedef struct                 //спутниковые данные
{
	char* conf;                //конфигурация
	char conflength;           //длина строки конфигурации

} pony_gnss;

typedef struct                 //шина
{
	int mode;                  //признак работы
	pony_imu* imu;
	pony_gnss* gnss;

	pony_data t;               //время

	char* conf;                //конфигурация
	char conflength;           //длина строки конфигурации
} pony_bus;

typedef struct
{
	pony_bus bus;

	void(**plugins)();         //указатель на массив указателей на плагины
	int pluginsNum;            //количество плагинов

	char* conf;                //конфигурация
	char conflength;           //длина строки конфигурации

	unsigned char exitplnum;   //номер плагина, вызвавшего завершившение работы
} pony_struct;

extern pony_struct pony;
