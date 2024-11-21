#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "gpio_port.h"
#include "oled_port.h"
#include "usart_port.h"
#include "adc_port.h"
#include "at32f413_board.h"
#include "app_MenuTask.h"
#include "app_MotorControlTask.h"

#include "customer_control.h"

#include "para_list.h"
#include "app_user_storage.h"
#include "apex_gc_port.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"	
#ifdef WDT_ENABLE
	#include "wdt.h"
	#include "event_groups.h"	
	extern EventGroupHandle_t  WDTEventGroup;
	extern TaskHandle_t  WDT_ManageTask_Handle;
#endif
	
extern TaskHandle_t  beepTemporaryTask_Handle;
extern 	QueueHandle_t  xQueueMotorControl;
extern QueueHandle_t   xQueueKeyMessage;//key	
extern QueueHandle_t  xQueueBeepMode;//beep
extern QueueHandle_t   xQueueMenuValue;
extern QueueHandle_t  xQueueBatValue;//batValue
extern QueueHandle_t xQueueApexValue;
extern SemaphoreHandle_t xSemaphoreDispRfresh;

static void Disp_ApexPreForSet(u8 value);
static void ApexBarFlashForSet(unsigned int systemTimeMs,unsigned char startFlag, short int apexValue);
#ifdef  APEX_FUNCTION_EBABLE
extern SemaphoreHandle_t xSemaphoreApexAutoCalibration;
#endif
	
#define MINIMUM_CW_CCW_ANGLE_GAP          80//minimum routor angle in  " motor_settings.mode=EndoModePositionToggle "	
	
#ifdef ZHX
#define MAX_MOTOR_SPEED   2200//2000//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滄鐐寸墪鑿愭い鎺嗗亾闁逞屽�?閸ㄥ墎绮�?鍫涗汗闁圭儤鎸鹃崢顏呯節閻㈤潧浠ч柛瀣�?閳�?�秹宕卞☉娆戝幈闁�?�函缍嗛崑鍡樻櫠閻㈠憡鐓欐い鏃傜摂濞堟�?�鏌嶉挊澶樻Ц闁宠绉归、妯款槺闂侇収鍨遍妵鍕閳╁喚�?�?悗瑙�?礀閵堢ǹ顕ｉ幘�?�藉亜闁告瑥顦褰掓⒒娴ｈ棄鍚瑰┑顔肩仛缁傚秵绂掔�?ｎ亞顦悗鍏夊亾闁告洦鍋�?崵鎴︽⒑绾懏褰ч梻鍕閹�?斥�?�閵忊€斥偓鐢告煥濠靛�?�鍑归柟鏌ョ畺閺岋綁骞橀崘鍙夊€銈冨妸閸庣敻骞冨▎鎾村�?��?�ゆ帒鍋嗛崯瀣⒒娴ｈ姤銆冮柣鎺炵畵楠炴垿宕堕鈧弸渚�?鏌熼崜�?�甯涢柡鍛倐閺屻劑�?ら崒娑橆�?(max 2450*6=14700)
#define MAX_TORQUE_UUPER_THRESHOLD   50//42//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滄鐐寸墪鑿愭い鎺嗗亾闁逞屽�?閸ㄥ墎绮�?鍫涗汗闁圭儤鎸鹃崢顏呯節閻㈤潧浠ч柛瀣�?閳�?�秹宕卞☉娆戝幈闁�?�函缍嗛崑鍡樻櫠閻㈠憡鐓欐い鏃傜摂濞堟�?�鏌嶉挊澶樻Ц闁宠绉归、妯款槺闂侇収鍨遍妵鍕閳╁喚�?�?悗瑙�?礀閵堢ǹ顕ｉ幘�?�藉亜闁告瑥顦褰掓⒒娴ｈ棄鍚瑰┑顔肩仛缁傚秵绂掔�?ｎ亞顦悗鍏夊亾闁告洦鍋�?崵鎴︽⒑绾懏褰ч梻鍕閹�?斥�?�閵忊€斥偓鐢告煥濠靛�?�鍑归柟鏌ョ畺閺岋綁骞橀崘鍙夊€銈冨妸閸庣敻骞冨▎鎾村�?��?�ゆ帒鍋嗛崯瀣⒒娴ｈ姤銆冮柣鎺炵畵楠炴垿宕堕鈧弸渚�?鏌熼崜�?�甯涢柡鍛倐閺屻劑�?ら崒娑橆�?(46.281g.cm/A~~0.454N.cm/A)(18.41mN.m or 1.841N.cm  1g.cm=0.00981N.cm=0.0981mN.m=0.0000981N.m)(闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵炩拃鍥х閺�?�牆澧界壕鍨归�?鈧崢鍓ф閹惧瓨�?村瀣唉缁�?�?�洪棃鈺�?Ф缂佽弓绮欓敐鐐剁疀濞戞瑦鍎柣鐔哥懃鐎氼剛澹曢鐐粹拺闂傚牊鑳嗚ぐ鎺戠？闁哄�??鍎查崐闈浳旈敐鍛�?�闁抽攱鍨块弻娑樷攽閸℃浼岄梺绋块缁绘垿濡甸崟�?�ｆ晣闁绘ɑ褰�?�?瀣⒑缂佹ü绶遍柛鐘崇〒缁鈽�?�Ο閿嬵潔濠电偛妫楃换鍡涘磻閹惧绡€婵﹩鍘鹃崢楣冩⒑鐠団€冲�?�閻㈩垱�?″畷婵嗏�?閸曨厾�?�曢梺鍛婁緱閸�??嫰鎮橀崣澶�?弿濠电姴鍟妵婵堚偓瑙勬处閸�?﹤鐣烽悢纰辨晝闁绘�?�娓规竟鏇㈡⒑閸撴彃浜濇繛鍙夌墱婢�?�洝銇愰幒鎾跺幐閻庡箍鍎�?鍛婄閹扮増鐓曢悗锝庡亝鐏忣厽銇勯锝囩疄闁�?�喗鐟╅、妤�?焵椤掑�?�鏁傞柨鐕傛�?4.0A闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿劍�?�?い�?�㈢箰鐓ゆい蹇撳閻�?厼顪�?妶鍡樼闁瑰啿閰ｅ畷婊堝级濞嗙偓瀵岄梺闈涚�?�閸燁偊宕濆�?�滀簻闁挎洖鍊瑰☉�?�銇�?銏㈢闁圭厧婀遍幉鎾礋椤愩倧绱￠梻鍌欑窔濞佳勭仚闂佺ǹ瀛╅悡锟犲箖濡や降鍋呴柛鎰ㄦ杹閹锋椽姊洪崨濠�?畵閻庢氨鍏樺畷鏇＄疀閺傝绨诲銈嗘尵婵挳宕㈤幘�?�界厽婵炴垵宕弸锔剧磼閻樺�?鈽�?�柍钘�?�槸閳�?�酣骞囬�?�鎴烽梻鍌氬�?峰ù鍥р枖閺囥垹闂柨鏇炲€哥粻顖炴煥閻曞倹瀚�187.71 g.cm=1.841N.cm=18.41mN.m=0.1841N.m)
#define HALF_MAX_TORQUE_UUPER_THRESHOLD   MAX_TORQUE_UUPER_THRESHOLD/2
#define MINIMUM_ROTOR_ANGLE          10//minimum routor angle in  " motor_settings.mode=EndoModePositionToggle "

//unsigned short int torque_list[16]={5,8,10,12,15,18, 20,22,25,30,35,40,42,50,55,60};//unit mN.m
  unsigned short int torque_list[21]={6,8,10,12,14,16,18, 20,22,24,26,28,30,32,35,40,42,45,50,55,60};//unit mN.m
short int speed_list[20]={100,150,200,250,300,350,400,450,500,550,600,800,1000,1200,1500,1800,2000,2200,2500};	
	
//	short int speed_list[20]={100,150,200,250,300,350,400,450,500,550,600,800,1000,1200,1500,1800,MAX_MOTOR_SPEED,2200,2500};	
	//7V
//unsigned short int torque_limit[22]={torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque65_Ncm ,\
//	torque65_Ncm ,torque65_Ncm ,torque60_Ncm ,torque50_Ncm ,torque45_Ncm  ,torque35_Ncm  ,torque20_Ncm  ,torque05_Ncm  ,torque05_Ncm  ,torque05_Ncm,torque05_Ncm,torque05_Ncm};//unit mN.m闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閸婄敻�?峰▎蹇擃仾缂佲偓閸愵亞纾兼い鏃囧亹閻掑憡銇勯姀�?╂垹缂撻悾宀�?�?欓柛褎顨呴弫褰掓⒒娴ｈ�?�甯涙慨濠咁潐缁傚秵绂掔�?ｎ亞锛涢梺鍝�?▉閸樹粙鍩涢幒妤佺厱閻忕偛澧介幊鍕磼娴ｅ搫顣肩紒缁樼⊕瀵板�?鈧綆鍋嗛ˇ浼存倵鐟欏�??纾�?�柛妤佸▕閻涱噣宕堕鈧痪褔鏌涢…鎴濇灈婵炲牄鍊曢埞鎴︽偐閸偅姣勬繝娈�?枤閺佸骞婂┑瀣�?鐟滃繑绋夊鍡愪簻闁哄稁鍋勬禒锕傛煟閹捐泛鏋涢柡宀�?鍠愬蹇涘�?�瑜嶉崺宀�?偠濮橆厼鍝烘慨濠冩そ瀹曨偊宕熼鈧▍銈夋⒑鐠団�?�?灈闁稿﹤鐏濋锝嗙節�?橆厽娅㈤梺缁樓圭亸娆撴晬濠婂啠鏀介柍钘�?�閻忋儵鏌曢崱蹇撲壕闂備胶�?椤戝棝骞戦崶褜鍤曞ù鐘�?儛閺佸啴鏌ｉ�?鍥ㄨ吂濠㈣娲栭埞鎴︻敊閺傘倓绶甸梺鍛婃尰缁�?挾鍒掗弬�?�?椽顢旈崨顖氬箰闂佽绻掗崑鐘活敋瑜庨幈銊╁磼閻愬�?鍘靛銈嗘煥閹碱偊鎮橀懠�?�藉亾鐟欏�??纾搁柛鏂跨Ф閹广垹鈹戠�?ｎ亞顦板銈�?箰濡盯鏁嶉悢鍏尖拻闁�?�本鑹鹃埀顒勵棑缁牊鎷呴搹鍦槸婵犵數濮村▔姘緞閹邦剛顔掗柣鐘叉穿鐏忔瑩藝瑜嶉埞鎴︻敊婵劒绮堕梺绋�?�儐閹搁箖鍩�?椤掍緡鍟忛柛锝庡櫍瀹曟垶绻濋崶鈺佺ウ濠碘�?�鍨甸崑鎰閸忛棿绻嗘い鏍ㄧ矊閸斿鏌ｉ敐鍥╁笡缂佺粯绻傞埢鎾诲垂椤旂�?浜堕梺鐓庣仌閸ャ劎鍘遍柟鍏肩暘閸斿骞�?�ィ鍐╃厱闁宠鍎虫禍鐐繆閻愵亜鈧牠宕洪崼銉ョ婵炲棗绶峰ú�?�勎ч柛銉㈡櫃缁�?��?�?�?妶鍡欏⒈闁�?�鍋ら幃锟犳偄閸忚偐鍘甸柡澶婄墕�?�т粙宕氶幍�?�藉仏婵ǹ鍩�?埛鎴︽煕濠靛棗�?�柛锝嗘そ閺岀喖顢欓悡搴樺亾閸噮鍤曞┑鐘宠�?�鎯熼梺鍐叉惈閸婂憡绂嶉悙鐑樷拺缂佸瀵у﹢鎵磼鐎ｎ偄鐏存い銏℃閺佹捇鏁撻敓锟�
//8V
unsigned short int torque_limit[22]={torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque65_Ncm ,\
	torque65_Ncm ,torque65_Ncm ,torque60_Ncm ,torque60_Ncm ,torque60_Ncm  ,torque50_Ncm  ,torque35_Ncm  ,torque20_Ncm  ,torque08_Ncm  ,torque06_Ncm,torque06_Ncm,torque06_Ncm};//unit mN.m闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閸婄敻�?峰▎蹇擃仾缂佲偓閸愵亞纾兼い鏃囧亹閻掑憡銇勯姀�?╂垹缂撻悾宀�?�?欓柛褎顨呴弫褰掓⒒娴ｈ�?�甯涙慨濠咁潐缁傚秵绂掔�?ｎ亞锛涢梺鍝�?▉閸樹粙鍩涢幒妤佺厱閻忕偛澧介幊鍕磼娴ｅ搫顣肩紒缁樼⊕瀵板�?鈧綆鍋嗛ˇ浼存倵鐟欏�??纾�?�柛妤佸▕閻涱噣宕堕鈧痪褔鏌涢…鎴濇灈婵炲牄鍊曢埞鎴︽偐閸偅姣勬繝娈�?枤閺佸骞婂┑瀣�?鐟滃繑绋夊鍡愪簻闁哄稁鍋勬禒锕傛煟閹捐泛鏋涢柡宀�?鍠愬蹇涘�?�瑜嶉崺宀�?偠濮橆厼鍝烘慨濠冩そ瀹曨偊宕熼鈧▍銈夋⒑鐠団�?�?灈闁稿﹤鐏濋锝嗙節�?橆厽娅㈤梺缁樓圭亸娆撴晬濠婂啠鏀介柍钘�?�閻忋儵鏌曢崱蹇撲壕闂備胶�?椤戝棝骞戦崶褜鍤曞ù鐘�?儛閺佸啴鏌ｉ�?鍥ㄨ吂濠㈣娲栭埞鎴︻敊閺傘倓绶甸梺鍛婃尰缁�?挾鍒掗弬�?�?椽顢旈崨顖氬箰闂佽绻掗崑鐘活敋瑜庨幈銊╁磼閻愬�?鍘靛銈嗘煥閹碱偊鎮橀懠�?�藉亾鐟欏�??纾搁柛鏂跨Ф閹广垹鈹戠�?ｎ亞顦板銈�?箰濡盯鏁嶉悢鍏尖拻闁�?�本鑹鹃埀顒勵棑缁牊鎷呴搹鍦槸婵犵數濮村▔姘緞閹邦剛顔掗柣鐘叉穿鐏忔瑩藝瑜嶉埞鎴︻敊婵劒绮堕梺绋�?�儐閹搁箖鍩�?椤掍緡鍟忛柛锝庡櫍瀹曟垶绻濋崶鈺佺ウ濠碘�?�鍨甸崑鎰閸忛棿绻嗘い鏍ㄧ矊閸斿鏌ｉ敐鍥╁笡缂佺粯绻傞埢鎾诲垂椤旂�?浜堕梺鐓庣仌閸ャ劎鍘遍柟鍏肩暘閸斿骞�?�ィ鍐╃厱闁宠鍎虫禍鐐繆閻愵亜鈧牠宕洪崼銉ョ婵炲棗绶峰ú�?�勎ч柛銉㈡櫃缁�?��?�?�?妶鍡欏⒈闁�?�鍋ら幃锟犳偄閸忚偐鍘甸柡澶婄墕�?�т粙宕氶幍�?�藉仏婵ǹ鍩�?埛鎴︽煕濠靛棗�?�柛锝嗘そ閺岀喖顢欓悡搴樺亾閸噮鍤曞┑鐘宠�?�鎯熼梺鍐叉惈閸婂憡绂嶉悙鐑樷拺缂佸瀵у﹢鎵磼鐎ｎ偄鐏存い銏℃閺佹捇鏁撻敓锟�
	
	#else 
#define MAX_MOTOR_SPEED    2450//moons =3000
#define MAX_TORQUE_UUPER_THRESHOLD  42// 50//moons(max 5.0N.cm)
#define HALF_MAX_TORQUE_UUPER_THRESHOLD   MAX_TORQUE_UUPER_THRESHOLD/2
#define MINIMUM_ROTOR_ANGLE          10//minimum routor angle in  " motor_settings.mode=EndoModePositionToggle "

short int speed_list[20]={100,150,200,250,300,350,400,450,500,550,600,800,1000,1200,1500,1800,2000,2200,MAX_MOTOR_SPEED};
//float torgue_list[13]={0.5,0.8,1.0,1.2,1.5,1.8,2.0,2.2,2.5,3.0,3.5,4.0,MAX_TORQUE_UUPER_THRESHOLD};//unit N.cm
unsigned short int torque_list[13]={5,8,10,12,15,18,20,22,25,30,35,40,MAX_TORQUE_UUPER_THRESHOLD};//unit mN.m
#endif

typedef enum {
		ADJUST_MOTOR_PARAM_PROGRAM_NUM = 0,
		ADJUST_MOTOR_PARAM_SPEED,          
		ADJUST_MOTOR_PARAM_TORQUE, 
		ADJUST_MOTOR_PARAM_DIRECTION,	
		ADJUST_MOTOR_PARAM_ANGLE_FORWARD , 
		ADJUST_MOTOR_PARAM_ANGLE_RESERVE , 
}ADJUST_select_NUM;

#define MENU_PARAM_ADD           0
#define MENU_PARAM_SUB          1

#define MINUTE_IDLE_TICKS           60000//1minute
#define MENU_IDLE_TICKS           MINUTE_IDLE_TICKS*3//5000//5s
#define HOME_PARAM_IDLE_TICKS            5000//5s
#define DEVICE_POWER_OFF_TICKS          MINUTE_IDLE_TICKS*3//3 minute

extern MotorStatus_TypeDef motor_status;
extern MotorSettings_TypeDef motor_settings;

//SYSTEM_MOTOR_PARAM sys_MotorParam[10]={0};
extern union Param_Union sys_param_un; 
extern union Motor_Para_Union motor_param_un; 

static void MenuPageTurns(unsigned char pageID);
/**
  * @brief 	TorqueRangeCheck
  * @param  value
  * @retval none
  */
static unsigned short TorqueRangeCheck (unsigned short value)
{
	unsigned short temp;
	temp=value;
	if(temp>MAX_TORQUE_UUPER_THRESHOLD) temp=MAX_TORQUE_UUPER_THRESHOLD;
	return temp;
}


/**
  * @brief 	MenuIdleTimeManage
  * @param  systemTime
  * @retval none
  */
static error_status  MenuIdleTimeManage(unsigned int systemTime,unsigned int targetTime)
{	
	error_status err;	
	static  unsigned int recTicks;
	err=ERROR;
	if(targetTime==0||recTicks>systemTime) 
	{
		recTicks=systemTime;		
	}
	else
	{		
		if(systemTime-recTicks>targetTime)
		{
			recTicks=systemTime;
			err=SUCCESS;
		}
	
	}	
	return err;
}
/**
  * @brief 	DEviceOffTimeManage
  * @param  systemTime
  * @retval none
  */
static error_status  DeviceOffTimeManage(unsigned int systemTime,unsigned int targetTime)
{	
	error_status err;	
	static  unsigned int recTicks;
	err=ERROR;
	if(targetTime==0||recTicks>systemTime) 
	{
		recTicks=systemTime;		
	}
	else
	{		
		if(systemTime-recTicks>targetTime)
		{
			recTicks=systemTime;
			err=SUCCESS;
		}		
	}	
	return err;
}

/**
  * @brief 	timerCountDown
  * @param  
  * @retval none
  */
static error_status timerCountDown(unsigned int systemTimeS,unsigned short int targetTimeS,unsigned char startFlag)
{
	error_status err;	
	static unsigned int recTimeS;
	static unsigned int realTimeS;
	err=ERROR;	
	if(startFlag==0)
	{
		recTimeS=systemTimeS;
		realTimeS=0;

		if(targetTimeS==0)
		{
			OLED_ShowString(114,40,"   ",16,1);		 
		}
		else
		{
			OLED_ShowNum(114,40,targetTimeS,2,16,1); 
			OLED_ShowChar(130,40,'S',16,1); 
		}
	}
	else 
	{		
		if((systemTimeS-recTimeS)!=realTimeS)
		{
			MenuIdleTimeManage(xTaskGetTickCount(),0);
			realTimeS=systemTimeS-recTimeS;
			if(realTimeS>targetTimeS)
			{					
				err=SUCCESS;
			}
			else 
			{	
				if(targetTimeS>10)
				{
					OLED_ShowNum(114,40,targetTimeS,2,16,1);
					OLED_ShowChar(130,40,'S',16,1); 		
				}
				else
				{
					if(targetTimeS==0)
					{						
						OLED_ShowString(114,40,"   ",16,1);						
					}
					else
					{
						OLED_ShowChar(114,40,' ',16,1); 						
						OLED_ShowNum(122,40,targetTimeS-realTimeS,1,16,1);	
						OLED_ShowChar(130,40,'S',16,1); 	
					}						
				}			
			}	
			xSemaphoreGive(xSemaphoreDispRfresh);				
		}			
	}
	return err;
}
/**
  * @brief 	MenuDevicePowerOff
  * @param  unsigned char feedDogFlag
  * @retval none
  */
void MenuDevicePowerOff(unsigned char feedDogFlag)
{
		if(feedDogFlag==2)//restore
		{
			default_para_write_buff();
		}
		App_MotorControl(MOTOR_MODE_STOP);
		#ifndef RTOS_UNUSED
//		taskENTER_CRITICAL();	
		__disable_irq();	
		write_para_judge();//save  date			
//		taskEXIT_CRITICAL();
			__enable_irq();
		vTaskDelay(2);//	1ms
		#else
		__disable_irq();	
		write_para_judge();//save  date			
			__enable_irq();
		delay_1ms(10);		
		#endif
		OLED_DisPlay_Off();
		device_power_off();//power off	
	#ifdef DEBUG_RTT
		SEGGER_RTT_WriteString(0,"power off\r\n");
	#endif	
		while(1) 
		{			
//			if(feedDogFlag!=0)
//			{//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡灞糕偓鎰佸悑閹肩补鈧�?顔愰梺鎼炲劜閹�?�啿�?忛搹鍦＜婵☆垵娅ｆ禒鎼佹煢閸愵厺鍚紒杈ㄦ�?椤撳ジ宕ㄩ鍏肩亷闂備�?�鎼惌澶岀礊娓氣偓閻涱噣宕堕渚囨濠电偞鍨堕敃鈺侇焽缂佹ü绻嗛柣鎰典簻閳ь剚娲滈幑銏ゅ箛閻�?�牆浜梺缁樻�?鐎垫帒顭�?弽�?�熺叄闊洦鍑瑰鎰偓瑙勬礀椤︾敻�??婚弴鐔虹闁割煈鍠栨慨鏇㈡⒑鐠囨煡鍙�?紒鐘崇墪椤繘鎼归悷鏉�?�嚙闂佸�?娲ㄩ崰鎰版偟閺冨牊鈷戦柣鐔告緲閺嗚京绱掔紒妯烘诞闁糕斁鍋撳銈嗗笒閸婂綊�?抽埡鍛厱婵☆垳�?村ú銈夋偂濠靛绠规繛锝庡�?婵¤偐绱掗悩宕団�?�闁靛洤瀚伴�?鎺�?�箣椤撶啘銉х磽娴ｇ懓鏁剧紒鐘虫�?椤繑绻濆顒傦紲濠电偛�?欓崝妤呭Χ閺�?�簱鏀介柣鎰▕濡茶绻涢懠�?�€鏋庨柣锝夋敱鐎靛ジ�??堕幋鐘垫澑闂備�?�鎼ˇ�?�炴倶濠靛鍚归柟瀵�?�仧缁♀偓闂佹眹鍨藉�?�鐗庨梻浣藉亹閹�?挻鏅堕悾灞藉灊濠电姵鍑归�?宥�?�煟閹邦剦鍤熼柛�?�?矙濮婅�?�娑甸崨顔兼�?缂備焦褰�?埀顒傚櫏閺嗛亶姊�?�崒娆戭�?闁圭⒈鍋婇獮濠呯疀濞戞鐣�?梺绋跨灱閸�??�?鎷戦悢鍏肩厪濠电偟鍋撳▍鍡涙煕鐎ｎ亝顥㈤柡灞剧〒娴狅箓宕滆�?濡插牆顪�?妶鍛寸崪闁瑰嚖鎷�?
				#ifdef WDT_ENABLE
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
				#endif
//			wdt_counter_reload();//wait  run_button_release_signal 
//			}
		}				 
}
/**
  * @brief 	MenuConfigMotorParam
  * @param  paramType
  * @retval none
  */

static error_status  MenuConfigMotorParam(unsigned char programNum,ADJUST_select_NUM selectNum,unsigned char addOrSub)
{		
	error_status err;
	err=ERROR;
	//if(programNum<10) //==10onlay apex mode
	if((programNum>5)&&programNum<10) //==10onlay apex mode//p0~p5,for bit 
	{		
		if(selectNum==ADJUST_MOTOR_PARAM_PROGRAM_NUM)
		{	//p0~p9
		}		
		else if(selectNum==ADJUST_MOTOR_PARAM_SPEED)//speed
		{	
			if(addOrSub==MENU_PARAM_ADD)
			{	
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum++;						
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum%=(spd600_Rpm_num+1);	
				}
				else 
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum++;						
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum%=(MAX_spd_Rpm_num+1);						
				}
			}
			else if(addOrSub==MENU_PARAM_SUB) 
			{//mini=150rpm
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
				{
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum==spd100_Rpm_num)
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum=spd600_Rpm_num;
					}
					else
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum--;	
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum%=(spd600_Rpm_num+1)	;	
					}
				}
				else
				{
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum==spd100_Rpm_num)
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum=MAX_spd_Rpm_num;
					}
					else
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum--;	
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum%=(MAX_spd_Rpm_num+1)	;	
					}
				}				
			}		
			if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum>torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
			{
			 motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum=torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
			}	
		}
		else if(selectNum==ADJUST_MOTOR_PARAM_TORQUE)//torque
		{			
			if(addOrSub==MENU_PARAM_ADD)
			{	
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum++;	
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum%=(MAX_torque_42_Ncm+1);
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum<torque20_Ncm) motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum=torque20_Ncm;
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum>torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
					 {
						 motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum=torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					 }	
				}		
				else if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)//max 2.0
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum++;	
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum%=(torque20_Ncm+1);
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum>torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
					 {
						 motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum=torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					 }							
				}
				else
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum++;	
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum%=(MAX_torque_42_Ncm+1);	
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum>torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum=torque06_Ncm;
					}	
				}	
						
			}
			else  if(addOrSub==MENU_PARAM_SUB)
			{	
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
				{
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum<=torque20_Ncm)
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum=MAX_torque_42_Ncm;
					} 
					else 
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum--;	
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum%=(MAX_torque_42_Ncm+1);
					}					
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum>torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum=torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					}	
				}																				
				else if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)//max 2.0
				{
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum<=torque06_Ncm)
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum=torque20_Ncm;
					}
					else
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum--;
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum%=(torque20_Ncm+1);							
					}							
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum>torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
					{
						 motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum=torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					}	
				}
				else 
				{
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum<=torque06_Ncm)
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum=MAX_torque_42_Ncm;
					}
					else
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum--;
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum%=(MAX_torque_42_Ncm+1);							
					}							
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum>torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
					{
						 motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum=torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					}	
				}					
			}			
		}
		else if(selectNum==ADJUST_MOTOR_PARAM_DIRECTION)//dir,0,1,2,3
		{
			if(addOrSub==MENU_PARAM_ADD)
			{
				motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir++;
				motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir%=Max_endoMode;						
			}
			else if(addOrSub==MENU_PARAM_SUB)
			{				
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir>EndoModePositionToggle)	
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir--;
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir%=Max_endoMode;
				}
				else 
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir=EndoModeTorqueATC;
				}				
			}
		}
		else if(selectNum==ADJUST_MOTOR_PARAM_ANGLE_FORWARD)//angle   forward
		{			
			if(addOrSub==MENU_PARAM_ADD)
			{				
				motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition+=MINIMUM_ROTOR_ANGLE;
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition>360) 
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition=10;	
				}	  				
			}
			else if(addOrSub==MENU_PARAM_SUB)
			{
				motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition-=MINIMUM_ROTOR_ANGLE;
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition<10) 
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition=360;
				}		
			}			
		}
		else if(selectNum==ADJUST_MOTOR_PARAM_ANGLE_RESERVE)//angle   reserve
		{			
			if(addOrSub==MENU_PARAM_ADD)
			{	
				motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition-=MINIMUM_ROTOR_ANGLE;
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition<-360) 
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition=-10;
				}
			}
			else if(addOrSub==MENU_PARAM_SUB)
			{
				motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition+=MINIMUM_ROTOR_ANGLE;
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition>-10) 
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition=-360;
				}
			}				
		}		
	}
	err=SUCCESS;
	return err;
}
/**
  * @brief 	MenuMotorParamUpdate
  * @param program, selectNum
  * @retval none
  */
static  error_status  MenuMotorParamUpdate(unsigned short int programNum)
{	
	error_status err;
	err=ERROR;
  	if(programNum<10) //==10 only apex mode
	{
		if(motor_param_un.system_motor_pattern[programNum].pNum!=programNum) //first
		{
			motor_param_un.system_motor_pattern[programNum].pNum=programNum;
			motor_param_un.system_motor_pattern[programNum].recTorqueThresholdNum=torque40_Ncm;//				
			motor_param_un.system_motor_pattern[programNum].forwardPosition = 30; 
			motor_param_un.system_motor_pattern[programNum].reversePosition = -150; 		
			motor_param_un.system_motor_pattern[programNum].torqueThresholdNum = torque40_Ncm;// (unit mN.m)(4.0 unit N.cm) 
			motor_param_un.system_motor_pattern[programNum].atcTorqueThresholdNum =	torque20_Ncm;	
			motor_param_un.system_motor_pattern[programNum].toggleSpeedNum =spd300_Rpm_num; 
		}		
		//.....//save to FLASH when power off	
		// MOTOR_SETTING_UPDATE
		motor_settings.autorev_mode=AutoReverseMode1;	
		if(motor_param_un.system_motor_pattern[programNum].dir==EndoModeTorqueATC)
		{
			motor_settings.mode=EndoModeSpeedForward;				
		}
		else
		{			
			motor_settings.mode =(eEndoMode) motor_param_un.system_motor_pattern[programNum].dir;	
		}	
		motor_settings.forward_position=motor_param_un.system_motor_pattern[programNum].forwardPosition;
		motor_settings.reverse_position=motor_param_un.system_motor_pattern[programNum].reversePosition;
		if(motor_param_un.system_motor_pattern[programNum].dir==EndoModeTorqueATC)
		{		
			if(motor_param_un.system_motor_pattern[programNum].atcTorqueThresholdNum>torque_limit[motor_param_un.system_motor_pattern[programNum].motorSpeedNum])
			{
				motor_param_un.system_motor_pattern[programNum].atcTorqueThresholdNum=torque_limit[motor_param_un.system_motor_pattern[programNum].motorSpeedNum];
			}				
			motor_settings.upper_threshold=torque_list[torque40_Ncm]*0.10f;//MAX_TORQUE_UUPER_THRESHOLD*0.10;			
			motor_settings.lower_threshold=torque_list[motor_param_un.system_motor_pattern[programNum].atcTorqueThresholdNum]*0.10f;//torque_list[motor_param_un.system_motor_pattern[programNum].torqueThresholdNum]*0.10;//闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊椤掑﹦绋忔繝銏ｅ煐閸旓箓�?繝鍥ㄧ厸鐎广儱楠搁�?鏍煟椤撶噥娈滈柡灞剧洴楠炲�?�?冨☉娆戜�?闂備礁鎼鍕濮樿泛钃熼柣鏃傚帶缁犳煡鏌熸�?�瀛樻�?婵炲牜鍘奸埞鎴﹀煡閸℃ぞ绨奸梺鎸庢磸閸ㄨ棄�?�ｆ繝�?�嵆闁靛繒濞€閸炶泛鈹戦悩缁樻�?婵炴潙鍊歌灋闁绘劕妯婂〒濠�?煏閸繃鍣界紒鐘卞嵆閺岋綁�??借閸�?�垻鈧鍠�?…鐑藉垂妤ｅ�?绠涘ù锝呮啞閸婎垰鈹戦悙鑸靛涧缂佸弶宕橀妵鎰板�?�閳哄喚娲告俊銈忕到閸燁垶鎮￠弴銏＄厵闂侇叏绠戞晶�?�剧磼閻欐瑥娲﹂悡鐔哥節閸偄濮�?柣蹇ラ�??閹便劍绻濋崟顓炵闂佺懓鍢查幊�?�€骞婇悙鍝勎ㄧ憸瀣焵椤掆偓閿曨亜顫忓ú�?�勪紶闁靛鍎涢敐澶�?厱闁哄啠鍋撻柣鐔村劦椤㈡岸鏁�?径濠傜獩婵犵數濮寸€氼噣�?侀崼婵冩斀妞ゆ梹鏋绘笟娑㈡煕閹惧娲存い銏＄懇瀹曞�?鈽�?�▎灞惧缂傚倸鍊烽悞锕傛�?闂佽绻�?畷�?�勫煘閹达富鏁婇柛婵嗗閸�??挸鈹戦崱鈺佹闂佸搫琚崕杈╃不閻熸噴褰掓晲閸涱喛纭�?闂佸憡蓱閹倸顫忛搹鍦煓闁�?ǹ瀛╅幏閬嶆⒑閹�?��?�鍝�?柡灞界Х椤т線鏌涢幘鏉戝摵闁诡啫鍐ｅ牚闁割偅绻勯崝锕�?�?�?妶鍡楃瑐婵炴潙娲︾粩鐔肺熼懖鈺冿紳閻庡箍鍎遍幏鎴︾叕椤掑倵鍋撳▓�?灈妞ゎ參鏀辨穱濠囧箹娴ｅ摜鍘搁梺绋挎湰閻喚鑺辨繝姘拻濞达絽鎲￠�?鐐电磼鐎ｎ偄鐏撮柡浣稿暣閺佸倿鎸婃径�?烘闂備礁鎲″ú锕傚垂闁秴鐓曢柟瀵�?�У閸犳劙鏌ｅΔ鈧悧鍡樼┍椤栨稐绻嗘い鎰剁到閻忔挳鏌＄仦鍓ь灱缂佺姵鐩�?�€骞橀崗澶婁�?�閻犳亽鍔�?�崑鎾斥枔閸喗鐝梺绋款儏閿曨�?�?伴鍢�?�喖宕�?�悡搴ｅ酱闂備�?�鎲￠悷銉┧�?挊澹╂盯�?㈤崗灏栨嫽婵炴挻鍩冮崑鎾寸�?�娴ｅ啿娲﹂崑瀣煕閳╁啨浜�?�繛鎴烆焸閺冨牆妞藉ù锝堫潐濞�?棝�?�绘担钘変汗閺�?�亪鏌涘锝�?壕缂傚倷娴�?�?�螞濞嗘挸�?�闁告洦鍨伴崘鈧梺闈涒康缁茶偐鍒掗崼鏇熲拺缂佸�?�欓崕鎰版煙閸涘﹥鍊愰柟顕€绠栭幃�?�堟寠�?�跺�?鈧偤�?�洪崘鍙夋儓闁挎洏鍎靛顐﹀Χ閸℃瑧鐦堥梺姹囧灲濞佳勭閳哄懏鐓欐繛鑼额唺缁ㄧ�?鈧灚�?�橀敃�?�堝箰婵犲啫绶炴俊�?�滃帶婵椽姊绘担鑺ョ《闁哥姵鎸�?�幈銊╁箻椤旂厧鍤戞繛鎾村焹閸�??捇鏌″畝鈧崰鏍嵁閹达箑绠涢梻�?熺⊕椤斿�?绻濈喊妯活潑闁�?�鎳橀�?鍐閵堝懓鎽曢梺鍝�?储閸ㄥ綊鏌�??崶銊х瘈闂傚牊绋掔粊鈺備繆椤愩倕浠滄い顏勫暣婵″爼宕ㄧ�?电ǹ�?�介梻浣烘嚀閸ゆ牠骞忛敓锟� int  param=5闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?锋俊鎼佸煛閸屾矮绨介梻浣呵归張�?�傜矙閹达富鏁傞柨鐕傛�??5闂傚倸鍊搁崐鎼佸磹閻戣姤鍤勯柤鍝ユ暩娴犳碍淇婇悙顏勨偓鏍垂閻㈢ǹ绠犻柟鐗堟緲缁犳煡鏌曡箛鏇烆�?屾繛绗哄姂閺屽秷顧侀柛鎾寸懇椤㈡岸鏁�?径妯绘櫇闂佹寧娲嶉崑鎾剁磼閻樺樊鐓奸柟�?�肩秺瀹曟儼顦�?紒鐙欏�?�浜滈柕澶涘�?缁犵偞鎱ㄦ繝鍐┿仢婵☆偄鍟埥澶婎潩椤掑姣囧┑鐘�?�暯濡插懘宕戦崨娣偓鍐╃節閸ャ�?鍋撴笟鈧�?�€宕奸悢鍛婎仧闂備浇娉曢崳锕傚�?閿燂�?5 闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?锋俊鎼佸煛閸屾矮绨介梻浣呵归張�?�傜矙閹达富鏁傞柨鐕傛�??-5 闂傚倸鍊搁崐鎼佸磹閻戣姤鍤勯柤鍝ユ暩娴犳碍淇婇悙顏勨偓鏍垂閻㈢ǹ绠犻柟鐗堟緲缁犳煡鏌曡箛鏇烆�?屾繛绗哄姂閺屽秷顧侀柛鎾寸懇椤㈡岸鏁�?径妯绘櫇闂佹寧娲嶉崑鎾剁磼閻樺樊鐓奸柟�?�肩秺瀹曟儼顦�?紒鐙欏�?�浜滈柕澶涘�?缁犵偞鎱ㄦ繝鍐┿仢婵☆偄鍟埥澶婎潩椤掑姣囧┑鐘�?�暯濡插懘宕戦崨娣偓鍐╃節閸ャ�?鍋撴笟鈧�?�€宕奸悢鍛婎仧闂備浇娉曢崳锕傚�?閿燂�?5闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?锋俊鎼佸煛閸屾矮绨介梻浣呵归張�?�傜矙閹达富鏁傞柨鐕傛�??
			if(motor_param_un.system_motor_pattern[programNum].atcTorqueThresholdNum>torque20_Ncm)
			{
				motor_settings.lower_threshold=torque_list[torque20_Ncm]*0.10f;
			}
		}
		else if(motor_param_un.system_motor_pattern[programNum].dir==EndoModePositionToggle)
		{
			motor_settings.upper_threshold=torque_list[motor_param_un.system_motor_pattern[programNum].recTorqueThresholdNum]*0.1f;//MAX_TORQUE_UUPER_THRESHOLD*0.10;//motor_param_un.system_motor_pattern[programNum].torqueThreshold*0.16;
			motor_settings.lower_threshold=torque_list[motor_param_un.system_motor_pattern[programNum].recTorqueThresholdNum]*0.06f;//MAX_TORQUE_UUPER_THRESHOLD*0.06;//闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊椤掑﹦绋忔繝銏ｅ煐閸旓箓�?繝鍥ㄧ厸鐎广儱楠搁�?鏍煟椤撶噥娈滈柡灞剧洴楠炲�?�?冨☉娆戜�?闂備礁鎼鍕濮樿泛钃熼柣鏃傚帶缁犳煡鏌熸�?�瀛樻�?婵炲牜鍘奸埞鎴﹀煡閸℃ぞ绨奸梺鎸庢磸閸ㄨ棄�?�ｆ繝�?�嵆闁靛繒濞€閸炶泛鈹戦悩缁樻�?婵炴潙鍊歌灋闁绘劕妯婂〒濠�?煏閸繃鍣界紒鐘卞嵆閺岋綁�??借閸�?�垻鈧鍠�?…鐑藉垂妤ｅ�?绠涘ù锝呮啞閸婎垰鈹戦悙鑸靛涧缂佸弶宕橀妵鎰板�?�閳哄喚娲告俊銈忕到閸燁垶鎮￠弴銏＄厵闂侇叏绠戞晶�?�剧磼閻欐瑥娲﹂悡鐔哥節閸偄濮�?柣蹇ラ�??閹便劍绻濋崟顓炵闂佺懓鍢查幊�?�€骞婇悙鍝勎ㄧ憸瀣焵椤掆偓閿曨亜顫忓ú�?�勪紶闁靛鍎涢敐澶�?厱闁哄啠鍋撻柣鐔村劦椤㈡岸鏁�?径濠傜獩婵犵數濮寸€氼噣�?侀崼婵冩斀妞ゆ梹鏋绘笟娑㈡煕閹惧娲存い銏＄懇瀹曞�?鈽�?�▎灞惧缂傚倸鍊烽悞锕傛�?闂佽绻�?畷�?�勫煘閹达富鏁婇柛婵嗗閸�??挸鈹戦崱鈺佹闂佸搫琚崕杈╃不閻熸噴褰掓晲閸涱喛纭�?闂佸憡蓱閹倸顫忛搹鍦煓闁�?ǹ瀛╅幏閬嶆⒑閹�?��?�鍝�?柡灞界Х椤т線鏌涢幘鏉戝摵闁诡啫鍐ｅ牚闁割偅绻勯崝锕�?�?�?妶鍡楃瑐婵炴潙娲︾粩鐔肺熼懖鈺冿紳閻庡箍鍎遍幏鎴︾叕椤掑倵鍋撳▓�?灈妞ゎ參鏀辨穱濠囧箹娴ｅ摜鍘搁梺绋挎湰閻喚鑺辨繝姘拻濞达絽鎲￠�?鐐电磼鐎ｎ偄鐏撮柡浣稿暣閺佸倿鎸婃径�?烘闂備礁鎲″ú锕傚垂闁秴鐓曢柟瀵�?�У閸犳劙鏌ｅΔ鈧悧鍡樼┍椤栨稐绻嗘い鎰剁到閻忔挳鏌＄仦鍓ь灱缂佺姵鐩�?�€骞橀崗澶婁�?�閻犳亽鍔�?�崑鎾斥枔閸喗鐝梺绋款儏閿曨�?�?伴鍢�?�喖宕�?�悡搴ｅ酱闂備�?�鎲￠悷銉┧�?挊澹╂盯�?㈤崗灏栨嫽婵炴挻鍩冮崑鎾寸�?�娴ｅ啿娲﹂崑瀣煕閳╁啨浜�?�繛鎴烆焸閺冨牆妞藉ù锝堫潐濞�?棝�?�绘担钘変汗閺�?�亪鏌涘锝�?壕缂傚倷娴�?�?�螞濞嗘挸�?�闁告洦鍨伴崘鈧梺闈涒康缁茶偐鍒掗崼鏇熲拺缂佸�?�欓崕鎰版煙閸涘﹥鍊愰柟顕€绠栭幃�?�堟寠�?�跺�?鈧偤�?�洪崘鍙夋儓闁挎洏鍎靛顐﹀Χ閸℃瑧鐦堥梺姹囧灲濞佳勭閳哄懏鐓欐繛鑼额唺缁ㄧ�?鈧灚�?�橀敃�?�堝箰婵犲啫绶炴俊�?�滃帶婵椽姊绘担鑺ョ《闁哥姵鎸�?�幈銊╁箻椤旂厧鍤戞繛鎾村焹閸�??捇鏌″畝鈧崰鏍嵁閹达箑绠涢梻�?熺⊕椤斿�?绻濈喊妯活潑闁�?�鎳橀�?鍐閵堝懓鎽曢梺鍝�?储閸ㄥ綊鏌�??崶銊х瘈闂傚牊绋掔粊鈺備繆椤愩倕浠滄い顏勫暣婵″爼宕ㄧ�?电ǹ�?�介梻浣烘嚀閸ゆ牠骞忛敓锟� int  par
		}
		else 
		{
			motor_settings.upper_threshold=torque_list[motor_param_un.system_motor_pattern[programNum].torqueThresholdNum]*0.10f;
			motor_settings.lower_threshold=torque_list[motor_param_un.system_motor_pattern[programNum].torqueThresholdNum]*0.06f;//60%		
		}				
		if(motor_param_un.system_motor_pattern[programNum].toggleSpeedNum>spd600_Rpm_num)
		{
			motor_param_un.system_motor_pattern[programNum].toggleSpeedNum=spd600_Rpm_num;
		}
		if(motor_param_un.system_motor_pattern[programNum].motorSpeedNum>MAX_spd_Rpm_num)
		{
			motor_param_un.system_motor_pattern[programNum].motorSpeedNum=MAX_spd_Rpm_num;
		}	
		motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[programNum].motorSpeedNum];
		motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[programNum].motorSpeedNum];	
		motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[programNum].toggleSpeedNum];//[motor_param_un.system_motor_pattern[programNum].motorSpeedNum];
	//to motor control	 
		App_MotorControl(MOTOR_SETTING_UPDATE);	
		err=SUCCESS;
	}	
	return err;
}

//==================LOGO PAGE======================================
static void OLED_Display_LOGO(unsigned short int logoNum)
{		
	//OLED_ShowString(38,16,"ENDOART",24,1); 	
	//OLED_ShowString(0,48,"Ver-1.0.020241028",16,1); 	
	logo_disppaly();
}
//==================HOME  PAGE=====================================
static void OLED_Display_motorDirection(unsigned short int dir)
{	
//	OLED_DrawCircle(136,24,19);
    main_dir_fill(112,16, dir);	
}
/**
  * @brief 	motor param
  * @param   programNum ,uint8_t select
  * @retval none
  */
static void OLED_disp_motor_param(SYSTEM_MOTOR_PARAM* motorParam,ADJUST_select_NUM selectNum )
{
	uint32_t temp,temp2;	
	if(motorParam->dir==EndoModePositionToggle)
	{
		temp=speed_list[motorParam->toggleSpeedNum];
	}
	else temp=speed_list[motorParam->motorSpeedNum];	
	if(temp<1000)	
	{	
		OLED_ShowNum(40,24,temp,3,16,(selectNum!=ADJUST_MOTOR_PARAM_SPEED));	
		OLED_ShowChar(32,24,' ',16,(selectNum!=ADJUST_MOTOR_PARAM_SPEED));	
	}
	else
	{
		OLED_ShowNum(32,24,temp,4,16,(selectNum!=ADJUST_MOTOR_PARAM_SPEED));	
	}	
	temp=torque_list[motorParam->torqueThresholdNum];		
	if(motorParam->dir==EndoModePositionToggle)
	{
		temp=torque_list[motorParam->recTorqueThresholdNum];	
	}	
 	else if( motorParam->dir==EndoModeTorqueATC)
	{
		temp=torque_list[motorParam->atcTorqueThresholdNum];	
	}	
	temp2=(temp/10);
	OLED_ShowNum(40,48,temp2,1,16,(selectNum!=ADJUST_MOTOR_PARAM_TORQUE));//int
	OLED_ShowChar(48,48,'.',16,(selectNum!=ADJUST_MOTOR_PARAM_TORQUE));
	temp2=temp%10;
	OLED_ShowNum(52,48,temp2,1,16,(selectNum!=ADJUST_MOTOR_PARAM_TORQUE));	
	if(motorParam->dir==EndoModePositionToggle)
	{
		OLED_ShowString(116,0,"REC",16,(selectNum!=ADJUST_MOTOR_PARAM_DIRECTION));
		//OLED_ShowString(48,44," REC ",16,(selectNum!=3));
		temp=abs(motorParam->reversePosition);		
		if(temp<100)
		{
			//OLED_ShowString(104,48,"  -",8,(selectNum!=5));
			OLED_ShowString(96,56,"   ",8,(selectNum!=ADJUST_MOTOR_PARAM_ANGLE_RESERVE));
			OLED_ShowNum(114,56,temp,2,8,(selectNum!=ADJUST_MOTOR_PARAM_ANGLE_RESERVE));
		}
		else
		{
			//OLED_ShowChar(110,48,'-',8,(selectNum!=5));
			OLED_ShowChar(102,56,' ',8,(selectNum!=ADJUST_MOTOR_PARAM_ANGLE_RESERVE));
			OLED_ShowNum(108,56,temp,3,8,(selectNum!=ADJUST_MOTOR_PARAM_ANGLE_RESERVE));
		}
		OLED_ShowChar(126,56,'/',8,1);
		temp=abs(motorParam->forwardPosition);	// angle
		if(temp<100)
		{			
			OLED_ShowNum(132,56,temp,2,8,(selectNum!=ADJUST_MOTOR_PARAM_ANGLE_FORWARD));
			OLED_ShowChar(144,56,' ',8,(selectNum!=ADJUST_MOTOR_PARAM_ANGLE_FORWARD));
		}
		else
		{
			OLED_ShowNum(132,56,temp,3,8,(selectNum!=ADJUST_MOTOR_PARAM_ANGLE_FORWARD));
		}			
	}
	else if(motorParam->dir==EndoModeTorqueATC){//ATC		
		OLED_ShowString(116,0,"ATC",16,(selectNum!=ADJUST_MOTOR_PARAM_DIRECTION));		
		temp=abs(motorParam->reversePosition);	// angle
		if(temp<100)
		{
//			OLED_ShowString(104,48,"  -",8,(selectNum!=ADJUST_MOTOR_PARAM_ANGLE_RESERVE));
			OLED_ShowString(96,56,"   ",8,(selectNum!=ADJUST_MOTOR_PARAM_ANGLE_RESERVE));
			OLED_ShowNum(114,56,temp,2,8,(selectNum!=ADJUST_MOTOR_PARAM_ANGLE_RESERVE));
		}
		else
		{
//		OLED_ShowChar(110,48,'-',8,(selectNum!=ADJUST_MOTOR_PARAM_ANGLE_RESERVE));
			OLED_ShowChar(102,56,' ',8,(selectNum!=ADJUST_MOTOR_PARAM_ANGLE_RESERVE));
			OLED_ShowNum(108,56,temp,3,8,(selectNum!=ADJUST_MOTOR_PARAM_ANGLE_RESERVE));
		}
		OLED_ShowChar(126,56,'/',8,1);
		temp=abs(motorParam->forwardPosition);
		if(temp<100)
		{
			OLED_ShowChar(144,56,' ',8,(selectNum!=ADJUST_MOTOR_PARAM_ANGLE_FORWARD));
			OLED_ShowNum(132,56,temp,2,8,(selectNum!=ADJUST_MOTOR_PARAM_ANGLE_FORWARD));
		}
		else
		{
			OLED_ShowNum(132,56,temp,3,8,(selectNum!=ADJUST_MOTOR_PARAM_ANGLE_FORWARD));
		}				
	}
	else if(motorParam->dir==EndoModeSpeedForward)
	 {//forward
		OLED_ShowString(116,0,"CW ",16,(selectNum!=ADJUST_MOTOR_PARAM_DIRECTION));	
		temp=360;	// angle	
		OLED_ShowNum(120,56,temp,3,8,1);		
		OLED_ShowString(96,56,"    ",8,1);
		OLED_ShowString(138,56,"  ",8,1);	
	}
	else if(motorParam->dir==EndoModeSpeedReverse){//reserve
		OLED_ShowString(116,0,"CCW",16,(selectNum!=ADJUST_MOTOR_PARAM_DIRECTION));		
		temp=360;
		OLED_ShowNum(120,56,temp,3,8,1);		
		OLED_ShowString(96,56,"   -",8,1);
		OLED_ShowString(138,56,"  ",8,1);			
	}		
		if((motorParam->dir)==EndoModeTorqueATC||(motorParam->dir)==EndoModePositionToggle)	
	{
		if(abs(motorParam->reversePosition)<abs(motorParam->forwardPosition))
		{
			OLED_Display_motorDirection(EndoModeTorqueATC);
		}
		else  OLED_Display_motorDirection(EndoModePositionToggle);
	}
	else OLED_Display_motorDirection(motorParam->dir);
}
/**
  * @brief 	home page
  * @param   programNum ,uint8_t select
  * @retval none
  */
static void OLED_Display_MAIN(unsigned short int programNum ,ADJUST_select_NUM select)
{	
	if(programNum<10) //programNum=10 only apex mode
	{
		 if(programNum==0)//P1-> GOLD 04
		{
			OLED_ShowString(32,0," GOLD 04 ",16,1);	
			OLED_ShowString(72,24,"rpm",16,1);		
		}	
		else if(programNum==1)//P2-> GOLD 06
		{
			OLED_ShowString(32,0," GOLD 06 ",16,1);
			OLED_ShowString(72,24,"rpm",16,1);
		}	
		else if(programNum==2)//P3-> BLUE 04
		{
			OLED_ShowString(32,0," BLUE 04 ",16,1);
			OLED_ShowString(72,24,"rpm",16,1);
		}	
		else if(programNum==3)//P4-> BLUE 06
		{
			OLED_ShowString(32,0," BLUE 06 ",16,1);
			OLED_ShowString(72,24,"rpm",16,1);
		}	
		else if(programNum==4)//P5-> PATH FILE
		{
			OLED_ShowString(32,0,"PATH FILE",16,1);
			OLED_ShowString(72,24,"rpm",16,1);
		}	
		else if(programNum==5)//P6-> EXPERT
		{
			OLED_ShowString(32,0,"  EXPERT ",16,1);
			OLED_ShowString(72,24,"rpm",16,1);
		}	
		else //if(programNum>5)//p7~P10
		{
			OLED_ShowString(32,0,"    ",16,1);//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿鏌涘┑鍡楊伂妞ゎ偓绠撻弻娑㈠籍閳ь剟宕归崸妤冨祦闁告劦鍠栭～鍛存煏閸繃鍣�?紒鎰⊕缁绘繈鎮介�?�娴躲垽�?楀闂寸敖婵″弶鍔欏鎾閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒冾潐濞叉﹢銆冮崨杈剧稏婵犲﹤鐗嗛悞�?亜閹哄棗浜惧銈嗘穿缂嶄線銆侀弴銏℃櫇闁逞屽墰缁鎳￠妶鍥╋紳婵炶�?缍€閸�??倿骞�?┑鍐╃€�?梺闈涚箞閸婃牠鍩涢幋锔藉�?闁�?�厽�?掓俊鍏肩箾閸涱喖�?嶉柡宀�?鍠栧畷娆撳Χ閸℃浼�
			OLED_ShowChar(64,0,'P',16,1);	
			OLED_ShowNum(72,0,programNum-5,1,16,1);//闂傚倸鍊搁崐鎼佸磹�?��??海鐭嗗〒�?�ｅ亾妤犵偞鐗犻、鏇㈠Χ閸屾矮澹曞┑�?�筋焽閸樠勬櫠椤�?偞鐓涢柛娑欐緲閻撴劗绱掗崒娑樼闁逞屽墾缂嶅�?�鍩�?椤掑�?�?块梻鍕�?楠炲牓濡搁妷銏℃杸闂佸湱鍋撳娆擃敂閿燂�?1-P4
			//OLED_ShowChar(22,4,' ',16,1);
			OLED_ShowString(80,0,"    ",16,1);
			OLED_ShowString(72,24,"rpm",16,1);			
		}		
		OLED_ShowChar(68,48,'N',16,1);	OLED_ShowChar(76,43,'.',16,1);	OLED_ShowString(80,48,"cm",16,1);	
		OLED_disp_motor_param(&motor_param_un.system_motor_pattern[programNum],select);
	}
}
//==================SET MENU MOTOR or APEX=====================================
/**
  * @brief 	SubmenuEntranceMotorOrApex
  * @param  systemTime
  * @retval none
  */
static void SubmenuEntranceMotorOrApex(unsigned char motorOrApex)
{	
	OLED_ShowString(36,8,"SETTING FOR",16,1);			
	OLED_ShowString(32,40,"MOTOR",16,(motorOrApex!=(0+2)));
	OLED_ShowString(96,40,"APEX",16,(motorOrApex!=(1+2)));		
}
/**
  * @brief 	HomePageHandle
  * @param  rec_Signal,enable
  * @retval none
  */
static void HomePageHandle(task_notify_enum rec_Signal,ADJUST_select_NUM selectNum)
{
	error_status err;	
	if(selectNum==ADJUST_MOTOR_PARAM_PROGRAM_NUM) 
	{
		if(rec_Signal==motor_setting_updata_signal&&sys_param_un.device_param.use_p_num<10)
		{
			err = MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);				
			if(err==SUCCESS)
			{
				OLED_Display_MAIN(sys_param_un.device_param.use_p_num ,ADJUST_MOTOR_PARAM_PROGRAM_NUM);//	
			}	
		}		
	}
	else 
	{		
		if(rec_Signal==add_button_press_signal)
		{	
			MenuConfigMotorParam(sys_param_un.device_param.use_p_num,selectNum,MENU_PARAM_ADD);									
		}	
		else if(rec_Signal==sub_button_press_signal)
		{	
			MenuConfigMotorParam(sys_param_un.device_param.use_p_num,selectNum,MENU_PARAM_SUB);	
		}			
		if(rec_Signal!=null_signal)
		{
			err = MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);				
			if(err==SUCCESS)
			{
				OLED_Display_MAIN(sys_param_un.device_param.use_p_num ,selectNum);//	
			}						
		}
		
	}	
}
/**
  * @brief 	SubmenuMotorParam
  * @param angleValue
  * @retval none
  */
static void SubmenuMotorParam( unsigned char cwOrCcw , short int angleValue)
{
	unsigned short int Value;
	Value=abs(angleValue);
	if(cwOrCcw==CCW_ANGLE)
	{	
		OLED_ShowString(44,8,"CCW ANGLE",16,1);	
		OLED_ShowChar(64,40,'-',16,1);
		if(Value<100)
		{	
			OLED_ShowNum(72,40,Value,2,16,1);
			OLED_ShowChar(88,40,' ',16,1);			
		}
		else
		{
			OLED_ShowNum(72,40,Value,3,16,1);	
		}
	}
	else  if(cwOrCcw==CW_ANGLE)
	{		
		OLED_ShowString(44,8,"CW  ANGLE",16,1);		
		OLED_ShowChar(72,40,' ',16,1);
		if(Value<100)
		{		
			OLED_ShowNum(72,40,Value,2,16,1);
			OLED_ShowChar(88,40,' ',16,1);
		}
		else
		{
			OLED_ShowNum(72,40,Value,3,16,1);
		}			
	}
}

/**
  * @brief 	SubmenuApexControlMode
  * @param angleValue
  * @retval none
  */
static void SubmenuApexControlMode( unsigned char apexMode,unsigned char enableFlag)
{
	if(apexMode==AUTO_START)//auto start 
	{		
		OLED_ShowString(40,8,"AUTO START",16,1);	
		OLED_ShowString(48,40,"OFF",16,(enableFlag!=0));	
		OLED_ShowString(88,40,"ON",16,(enableFlag!=1));
	}
	else if(apexMode==AUTO_STOP)//auto stop 
	{
		OLED_ShowString(40,8,"AUTO  STOP",16,1);	
		OLED_ShowString(48,40,"OFF",16,(enableFlag!=0));	
		OLED_ShowString(88,40,"ON",16,(enableFlag!=1));
	}
	else if(apexMode==APICAL_ACTION)//auto rec or stop
	{
		OLED_ShowString(28,8,"APICAL ACTION",16,1);	
		OLED_ShowString(32,40,"OFF",16,(enableFlag!=0));	
		OLED_ShowString(64,40,"REV",16,(enableFlag!=1));//REV
		OLED_ShowString(96,40,"STOP",16,(enableFlag!=2));
	}	
}
/**
  * @brief 	SubmenuApexControlMode
  * @param angleValue
  * @retval none
  */
static void SubmenuApexOffeset(	unsigned short int offeset)
{
		Disp_ApexPreForSet(60);		
		ApexBarFlashForSet(0,0,0);
}
//==================handness=====================================
/**
  * @brief 	SubmenuHandness
  * @param  systemTime
  * @retval none
  */
static void SubmenuHandness(unsigned char handness)
{	
//	OLED_ShowString(32,0,"Handness",24,1);			
//	OLED_ShowString(20,36,"Left",24,(handness!=0));			
//	OLED_ShowString(80,36,"Right",24,(handness!=1));	
		OLED_ShowString(48,8,"HANDNESS",16,1);			
		OLED_ShowString(40,40,"LEFT",16,(handness!=0));			
		OLED_ShowString(80,40,"RIGHT",16,(handness!=1));
}
//==================calibration=====================================
/**
  * @brief 	SubmenuCalibration
  * @param  flag
  * @retval none
  */
static void SubmenuCalibration(unsigned char flag)
{	
	if(flag==0||flag==1)
	{
		OLED_ShowString(16,8,"AUTO CALIBRATION",16,1);
		OLED_ShowString(22,40,"NO",16,(flag!=0));	
		OLED_ShowString(68,40,"YES",16,(flag!=1));	
	}
	else if(flag==2) 
	{
		OLED_ShowString(16,8," CALIBRATING... ",16,1);		
		OLED_ShowString(0,40,"   KEEP NO-LOAD     ",16,1);
	}
	else if(flag==3)
	{
		OLED_ShowString(16,8,"AUTO CALIBRATION",16,1);
		OLED_ShowString(0,40,"      FINISHED      ",16,1);
	}
	else
	{
		OLED_ShowString(16,8,"AUTO CALIBRATION",16,1);
		OLED_ShowString(0,40,"        FAILED      ",16,1);
	}
}

//==================restore default=====================================
/**
  * @brief 	SubmenuRestoreParam
  * @param  systemTime
  * @retval none
  */
static void SubmenuRestoreParam(unsigned char flag)
{	
	if(flag<2)
	{
		OLED_ShowString(8,8," RESTORE SETTINGS ",16,1);
		OLED_ShowString(22,40,"NO",16,(flag!=0));
		OLED_ShowString(68,40,"YES",16,(flag!=1));
	}
	else
	{
//		OLED_ShowString(16,8,"Restore Settings",16,1);
		 OLED_ShowString(8,8,"RESTORE SUCCESSFUL",16,1);
		OLED_ShowString(0,40,"  POWER OFF DEVICE  ",16,1);	
	}		
}

/**
  * @brief 	SubmenuPageHandle
  * @param  systemTime
  * @retval none
  */
static  void SubmenuPageHandle(unsigned short int subPageID,unsigned short int signal,unsigned int systemTime,unsigned char firstFlag)
{
	error_status err;
	static unsigned char secendNum,retCaliFanish=0;		
//	static unsigned char motorOrApex=0;	
	static unsigned int recSystemTime;
	static unsigned char blinkFlag;
	if(firstFlag==0)	
	{		
		if(subPageID==HANDEDNESS)
		{
			secendNum=sys_param_un.device_param.use_hand;	
		}		
		else 
		{
			secendNum=0;
			retCaliFanish=0;	
			recSystemTime =	systemTime;	
			timerCountDown(systemTime/1000,0,0)	;//start 10 			
		}		
		if(signal==motor_setting_updata_signal)	
		{		
			if(subPageID==CW_ANGLE||subPageID==CCW_ANGLE)	
			{
				SubmenuMotorParam( CW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition);					
				SubmenuMotorParam( CCW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition);	
			}			
			MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);	
		}			
	}
	else
	{	
		switch(subPageID)
		{		
			#ifdef APEX_FUNCTION_EBABLE	
			case SETTING_FOR:	 
				if(systemTime-recSystemTime>500)	
				{
					recSystemTime=systemTime;
					blinkFlag=(blinkFlag==0)?2:0;
					SubmenuEntranceMotorOrApex((firstFlag-1)|blinkFlag);
				}	
				if(signal==add_button_press_signal||signal==sub_button_press_signal)
				{
					SubmenuEntranceMotorOrApex((firstFlag-1)|blinkFlag);
				}	
			break;			
			case AUTO_START:	
				 if(signal==add_button_press_signal||signal==sub_button_press_signal)
				{		
					sys_param_un.device_param.auto_start_flag++;							
					sys_param_un.device_param.auto_start_flag%=2;
					SubmenuApexControlMode( AUTO_START,sys_param_un.device_param.auto_start_flag);		
				}		
			break;	
			case AUTO_STOP:
				if(signal==add_button_press_signal||signal==sub_button_press_signal)
				{	
					sys_param_un.device_param.auto_stop_flag++;							
					sys_param_un.device_param.auto_stop_flag%=2;
					SubmenuApexControlMode( AUTO_STOP,sys_param_un.device_param.auto_stop_flag);		
				}	
			break;	
			case APICAL_ACTION:			
				if(signal==add_button_press_signal||signal==sub_button_press_signal)
				{
					sys_param_un.device_param.apical_action_flag++;
					sys_param_un.device_param.apical_action_flag%=3;			
					SubmenuApexControlMode( APICAL_ACTION,sys_param_un.device_param.apical_action_flag);		
				}		
			break;
			case 	APICAL_OFFESET:				
				if(signal==sub_button_press_signal)
				{
					sys_param_un.device_param.ref_tine++;
					sys_param_un.device_param.ref_tine%=13;			
					SubmenuApexOffeset( sys_param_un.device_param.ref_tine);	
				}
				else if(signal==add_button_press_signal)
				{
					if(sys_param_un.device_param.ref_tine==0)
					{
						sys_param_un.device_param.ref_tine=12;
					}
					else 
					{
						sys_param_un.device_param.ref_tine--;
						sys_param_un.device_param.ref_tine%=13;		
					}
					SubmenuApexOffeset( sys_param_un.device_param.ref_tine);	
				}
				else
				{						
					ApexBarFlashForSet(xTaskGetTickCount(),1, 30);
				}	
			break;
			#endif
			case CW_ANGLE:
				if(signal==add_button_press_signal)
				{						
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition+=MINIMUM_ROTOR_ANGLE;
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition>360) motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition=MINIMUM_ROTOR_ANGLE;
					SubmenuMotorParam( CW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition);
				}
				else if(signal==sub_button_press_signal)
				{
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition<MINIMUM_ROTOR_ANGLE+MINIMUM_ROTOR_ANGLE) motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition=360;
					else motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition-=MINIMUM_ROTOR_ANGLE;	
					SubmenuMotorParam( CW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition);						
				}	
			break;	
			case CCW_ANGLE:
				if(signal==add_button_press_signal)
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition-=MINIMUM_ROTOR_ANGLE;
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition<-360) motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition=-MINIMUM_ROTOR_ANGLE;
					SubmenuMotorParam( CCW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition);	
				}
				else if(signal==sub_button_press_signal)
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition+=MINIMUM_ROTOR_ANGLE;	
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition>-MINIMUM_ROTOR_ANGLE)	
					{	
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition=-360;	
					}			
					SubmenuMotorParam( CCW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition);	
				}									
			break;			
			case HANDEDNESS:	
				if(signal==add_button_press_signal||signal==sub_button_press_signal)
				{	
					sys_param_un.device_param.use_hand++;
					sys_param_un.device_param.use_hand%=2;	
					SubmenuHandness(sys_param_un.device_param.use_hand);
				}					
			break;
			case AUTO_CALIBRATION:
				if(signal==add_button_press_signal||signal==sub_button_press_signal)
				{	
					secendNum++;//on or off
					secendNum%=2;	
					SubmenuCalibration(secendNum);
				}
				if(secendNum==0)
				{
					timerCountDown(systemTime/1000,10,0);				
				}
				else if(secendNum==1)
				{						
					err=timerCountDown(systemTime/1000,10,1)	;//start 10 
					if(err==SUCCESS)	
					{
						secendNum=2;//start
						retCaliFanish=0;
						SubmenuCalibration(2);
//						xSemaphoreGive(xSemaphoreApexAutoCalibration);							
					}	
				}
				else if(secendNum==2)
				{
					vTaskDelay(35);//wait apex calibration
					secendNum=3;
				}
				else if(secendNum==3)
				{		//start								
							
					#ifdef WDT_ENABLE							
					vTaskSuspend(WDT_ManageTask_Handle);								
					Wdt_Reset(MAX_WDT_FEED_TIME_MS*32);	//MAX_WDT_FEED_TIME_MS*32=9600mS						
					#endif 						
					retCaliFanish=App_MotorControl(MOTOR_MODE_SEARCH_ANGLE);						
					#ifdef WDT_ENABLE
					Wdt_Init();												
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);		
					vTaskResume(WDT_ManageTask_Handle);																
					#endif				
					if(retCaliFanish==0)
					{//	disp indicate	
						SubmenuCalibration(4);		
					}
					else
					{	//successful								
						SubmenuCalibration(3);
					}	
					xSemaphoreGive(xSemaphoreDispRfresh);	
					MenuIdleTimeManage(systemTime,0);	
					DeviceOffTimeManage(systemTime,0);							
					#ifdef  APEX_FUNCTION_EBABLE							
					xSemaphoreGive(xSemaphoreApexAutoCalibration);
					#endif								
					#ifdef WDT_ENABLE																		
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
					vTaskDelay(MAX_WDT_FEED_TIME_MS);//wait wdt dog reset							
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
					vTaskDelay(MAX_WDT_FEED_TIME_MS);//wait wdt dog reset
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);	
					vTaskDelay(MAX_WDT_FEED_TIME_MS);//wait wdt dog reset
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);	
					#else
					vTaskDelay(900);//wait wdt dog reset			
					#endif															
					secendNum=4;//end								
				}		
				else
				{
				//wait idle tiem
					signal=MENU_HOME_PAGE;//exit
					xQueueSend(xQueueMenuValue, &signal, 0);
				}	
				break;
				case RESTORE_DEFAULT_SETTING:
					if(signal==add_button_press_signal||signal==sub_button_press_signal)
					{	
						secendNum++;//on or off
						secendNum%=2;	
						SubmenuRestoreParam(secendNum);						
					}
					if(secendNum==0)
					{
						timerCountDown(systemTime/1000,10,0);
					}
					else if(secendNum==1)
					{
						err=timerCountDown(systemTime/1000,10,1)	;//start 10 
						if(err==SUCCESS)	
						{	
							SubmenuRestoreParam(2);
							xSemaphoreGive(xSemaphoreDispRfresh);	
							secendNum=2;	
							MenuIdleTimeManage(xTaskGetTickCount(),0);	
							#ifdef WDT_ENABLE																		
							xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
							vTaskDelay(MAX_WDT_FEED_TIME_MS);//wait wdt dog reset							
							xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
							vTaskDelay(MAX_WDT_FEED_TIME_MS);//wait wdt dog reset
							xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
							vTaskDelay(MAX_WDT_FEED_TIME_MS);//wait wdt dog reset
							xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);							
							#else
							vTaskDelay(900);//wait wdt dog reset			
							#endif
						}								
					}
					else
					{						
						MenuDevicePowerOff(2);															
					}						
					break;
			default:
			break;		 
	}	
	if(signal!=null_signal)
	{						
		if(subPageID==CW_ANGLE)
		{
			SubmenuMotorParam( CW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition);
			MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);							
		}
		else  if(secendNum==CCW_ANGLE)
		{
			SubmenuMotorParam( CCW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition);	
			MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);
		}		
	}
}	
	
}
/**
  * @brief 	SubmenuPageTurns
  * @param  systemTime
  * @retval none
  */
static  void SubmenuPageTurns(unsigned short int subPageID)
{		
	OLED_Clear_no_fresh();
	SubmenuPageHandle(subPageID,null_signal,0,0);//init
	switch(subPageID)
	{	
		#ifdef APEX_FUNCTION_EBABLE	
		case SETTING_FOR:				
			if(sys_param_un.device_param.use_p_num==10&&sys_param_un.device_param.apexFunctionLoad!=0)	
			{
				SubmenuEntranceMotorOrApex(1);	
			}	
			else 
			{
				SubmenuEntranceMotorOrApex(0);	
			}	
//			SubmenuEntranceMotorOrApex(motorOrApexSetFlag);		
		break;			
		case AUTO_START:
			SubmenuApexControlMode( AUTO_START,sys_param_un.device_param.auto_start_flag);
		break;	
		case AUTO_STOP:
			SubmenuApexControlMode( AUTO_STOP,sys_param_un.device_param.auto_stop_flag);
		break;	
		case APICAL_ACTION:
			SubmenuApexControlMode( APICAL_ACTION,sys_param_un.device_param.apical_action_flag);
		break;
		case APICAL_OFFESET:
			SubmenuApexOffeset(	sys_param_un.device_param.ref_tine);
		break;
		#endif
		case CW_ANGLE:
			SubmenuMotorParam( CW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition);
		break;	
		case CCW_ANGLE:
			SubmenuMotorParam( CCW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition);
		break;		
		case HANDEDNESS:	
			SubmenuHandness(sys_param_un.device_param.use_hand);			
		break;
		case AUTO_CALIBRATION:
			SubmenuCalibration(0);
		break;
		case RESTORE_DEFAULT_SETTING:
			SubmenuRestoreParam(0);
		break;	
		default:
		break;		 
	}	
}

//==================DISP in work mode  MOTOR AND APEX   picture=====================================
/**
  * @brief 	disp_TorqueOnly
  * @param   u8 x,u8 y,u8 torqueValue 
  * @retval none
  */
static void Disp_TorqueOnly(u8 x,u8 y ,unsigned short int torqueValue)
{
	unsigned short int temp;	
	temp=TorqueRangeCheck(torqueValue)/10;
	OLED_ShowNum(x,y,temp,1,24,1);
	OLED_ShowChar(x+12,y,'.',24,1);
	temp=TorqueRangeCheck(torqueValue)%10;
	OLED_ShowNum(x+20,y,temp,1,24,1);	
		
}
/**
  * @brief 	TorqueProgressBar
  * @param   u8 x,u8 y,start coord 
  * @retval none
  */
static void Disp_TorqueProgressBar(u8 x,u8 y ,u8 value)
{
	u8 i,j,x1,y1;
	//num
	x1=x;
	y1=y;
	for(i=0;i<5;i++)
	{
		j=5-i;	
		if(j>3)
		{			
			OLED_ShowNum(x1-3+i*25,y1+4,j,1,8,1);//>4 
			OLED_DrawPoint(x1+3+i*25, y1+9,1);
			OLED_DrawPoint(x1+4+i*25, y1+9,1);
			OLED_DrawPoint(x1+3+i*25, y1+10,1);
			OLED_DrawPoint(x1+4+i*25, y1+10,1);		
			OLED_ShowNum(x1+5+i*25,y1+4,0,1,8,1);			
		}		
		else
		{			
			OLED_ShowNum(x1+i*25,y1+4,j,1,8,1);			
			OLED_DrawPoint(x1+7+i*25, y1+9,1);
			OLED_DrawPoint(x1+8+i*25, y1+9,1);
			OLED_DrawPoint(x1+7+i*25, y1+10,1);
			OLED_DrawPoint(x1+8+i*25, y1+10,1);		
			OLED_ShowNum(x1+9+i*25,y1+4,0,1,8,1);
		}				
	}	
	x1=x;
	y1=y+13;
	for(i=0;i<2;i++)	//	  
	{
		for(j=0;j<5;j++)
		{
			if(j<2)
			{
				OLED_DrawPoint(x1+3+j*25, y1+i,1);
				OLED_DrawPoint(x1+4+j*25, y1+i,1);
			}
			else
			{
				OLED_DrawPoint(x1+7+j*25, y1+i,1);
				OLED_DrawPoint(x1+8+j*25, y1+i,1);		
			}	
		}
	  disp_DrawRow(x1,y1+i+1,120,1);		
	}	
}
/**
  * @brief 	TorqueProgressBarFlash
  * @param  systemTimeMs, startFlag 
  * @retval none
  */
static void TorqueProgressBarFlash(unsigned int systemTimeMs,unsigned char startFlag,unsigned short torqueValue)
{
	static unsigned int recTimeMs;
	static unsigned char blinkFlag;//
	static unsigned short torqueReference,maxTorqueNum;
	unsigned char i,j,x,y,m,torqueNum;	
	
	x=52;//40+16-8;//bar
	y=18;//16+2;		
	if(startFlag==0)//preparation work
	{
		recTimeMs=systemTimeMs;
		blinkFlag=0;	
		torqueValue=TorqueRangeCheck(torqueValue);
		if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
		{
			torqueValue=TorqueRangeCheck(torque_list[MAX_torque_42_Ncm]);
		}
		else if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)
		{
			if(torqueValue>torque_list[torque20_Ncm])
			{
				torqueValue=torque_list[torque20_Ncm];
			}			
		}	
		maxTorqueNum=(torque_list[MAX_torque_42_Ncm]-30)*2/5+13;	
   		if(maxTorqueNum>21)		 maxTorqueNum=21;	
		if(torqueValue>50)
		{
			torqueReference=maxTorqueNum;
		}
		else  if(torqueValue>30)
		{
			torqueReference=(torqueValue-30)*2/5+13;
		}
		else
		{
			if(torqueValue<5)
			{
				torqueReference=0;
			}
			else
			{
				torqueReference=((torqueValue-5)>>1);
			}
		}
		if(torqueReference>maxTorqueNum) torqueReference=maxTorqueNum;		
		i=torqueReference;		
		j=i*5;
		if(i<7)
		{
//			disp_DrawColumn(x+100-j,y,12+i,0);
			disp_DrawColumn(x+101-j,y+1,12+i,1);
			disp_DrawColumn(x+102-j,y,14+i,1);
			disp_DrawColumn(x+103-j,y+1,12+i,1);	
//			disp_DrawColumn(x+104-j,y,12+i,0);
		}
		else if(i<13)	
		{
//			disp_DrawColumn(x+100-j,y,i*3,0);
			disp_DrawColumn(x+101-j,y+1,i*3,1);
			disp_DrawColumn(x+102-j,y,i*3+2,1);
			disp_DrawColumn(x+103-j,y+1,i*3,1);	
//			disp_DrawColumn(x+104-j,y,i*3,0);
		}	
		else
		{
			j=i*6-11;
//		disp_DrawColumn(x+100-j,y,40,0);
			disp_DrawColumn(x+101-j,y+2,36,1);
			disp_DrawColumn(x+102-j,y+1,38,1);
			disp_DrawColumn(x+103-j,y,40,1);
			disp_DrawColumn(x+104-j,y+1,38,1);
			disp_DrawColumn(x+105-j,y+2,36,1);	
		}			
	}	
	else 
	{					
		if(systemTimeMs-recTimeMs>500)
		{			
			torqueValue = TorqueRangeCheck(torqueValue);					
			if(torqueValue>30)
			{
				torqueNum=(torqueValue-30)*2/5+13;
			}
			else
			{
				if(torqueValue<5)
				{
					torqueNum=0;
				}
				else
				{
					torqueNum=(torqueValue-5)>>1;//offset
				}
			}						
			if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir!=EndoModeTorqueATC)
			{
//				if(torqueNum>torqueReference)//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?節瀹曞爼鍩℃担鍦簴缂傚倷鑳舵繛鈧紒鐘崇墵瀵鈽夐�?�鐘栥劍銇�?�?鍌氬付�?�ゎ偀鏅濈槐鎾存�?�閹绘帊澹曢柣搴″帨閸�??捇鏌涢弴銊ュ�?闁告ɑ鎹囧铏光偓鍦У閵嗗啰绱掗埀顒佺瑹閳ь剙�?�ｉ崘娴�?瀻闁规�?�鏅欑花濠氭椤愩垺澶勯柟鎼佺畺瀹曟椽鏁愰崱�?哄伎婵犵數濮�?幊蹇涱敂閻�?�粯鐓欓柣鐔告緲椤忣厼鈹戦埄鍐╁�?�?┑锛�?厴閺佸倿鏌ㄩ姘婵°倧绲介崯顖炲煕閹达附鐓曟繝闈涙椤忔挳鏌涙惔锛勭闁靛洤瀚伴、�?�€�?╅幓鎺戠闂備胶�?笟妤呭窗濞戞氨涓嶆繛鎴欏灩缁秹鏌熺€电ǹ鈧偉顦归柡宀�?鍠栭幃鈩冩償閿濆棙鍠�?紓鍌欑椤戝棝宕濆Δ鍛闁圭儤顨嗛弲鎼佹煥閻曞倹瀚�
//				{
//					torqueNum=torqueReference;
//			}		
			}	
			if(torqueNum>maxTorqueNum) torqueNum=maxTorqueNum;						
			for(i=0;i<22;i++)
			{
				m=(i<torqueNum)?1:0;
				j=i*5;					
				if(i<7)
				{
//					disp_DrawColumn(x+100-j,y,12+i,0);
					disp_DrawColumn(x+101-j,y+1,12+i,m);
					disp_DrawColumn(x+102-j,y,14+i,m);
					disp_DrawColumn(x+103-j,y+1,12+i,m);
//					disp_DrawColumn(x+104-j,y,12+i,0);							
				}
				else if(i<13)
				{					
//					disp_DrawColumn(x+100-j,y,i*3,0);
					disp_DrawColumn(x+101-j,y+1,i*3,m);
					disp_DrawColumn(x+102-j,y,i*3+2,m);
					disp_DrawColumn(x+103-j,y+1,i*3,m);
//					disp_DrawColumn(x+104-j,y,i*3,0);			
				}
				else
				{
					j=6*i-11;
//					disp_DrawColumn(x+100-j,y,40,0);
					disp_DrawColumn(x+101-j,y+2,36,m);
					disp_DrawColumn(x+102-j,y+1,38,m);
					disp_DrawColumn(x+103-j,y,40,m);
					disp_DrawColumn(x+104-j,y+1,38,m);
					disp_DrawColumn(x+105-j,y+2,36,m);			
				}		
			}	
				//torque//test
//				Disp_TorqueOnly(124,40,torqueValue);
//				if(torqueValue>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]	)

//				{
//					Disp_TorqueOnly(124,40,torqueValue);
//				}
//				else
//				{
//					Disp_TorqueOnly(124,40,torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]);
//				}								
			recTimeMs=systemTimeMs;	
			blinkFlag=(blinkFlag)?0:1;				
			i=torqueReference;
			j=i*5;
			if(i<7)
			{
//				disp_DrawColumn(x+100-j,y,12+i,0);
				disp_DrawColumn(x+101-j,y+1,12+i,blinkFlag);
				disp_DrawColumn(x+102-j,y,14+i,blinkFlag);
				disp_DrawColumn(x+103-j,y+1,12+i,blinkFlag);
				disp_DrawColumn(x+104-j,y,12+i,0);		
			}
			else if(i<13)
			{
//				disp_DrawColumn(x+100-j,y,i*3,0);
				disp_DrawColumn(x+101-j,y+1,i*3,blinkFlag);
				disp_DrawColumn(x+102-j,y,i*3+2,blinkFlag);
				disp_DrawColumn(x+103-j,y+1,i*3,blinkFlag);
			}		
			else
			{
				j=i*6-11;
//				disp_DrawColumn(x+100-j,y,38,0);
				disp_DrawColumn(x+101-j,y+2,36,blinkFlag);
				disp_DrawColumn(x+102-j,y+1,38,blinkFlag);
				disp_DrawColumn(x+103-j,y,40,blinkFlag);
				disp_DrawColumn(x+104-j,y+1,38,blinkFlag);
				disp_DrawColumn(x+105-j,y+2,36,blinkFlag);	
			}				
//			xTaskNotifyGive(beepTemporaryTask_Handle);					
				xSemaphoreGive(xSemaphoreDispRfresh);	
		}
	}
}

/**
  * @brief 	TorqueProgressBarFlash
  * @param  systemTimeMs, startFlag 
  * @retval none
  */
//startFlag0,reset;1
static void ApexProgressBarFlash(unsigned int systemTimeMs,unsigned char startFlag, short int apexValue)
{
	static unsigned int recTimeMs,blinkTimeMs;
	static unsigned short blink,recApexValue,temp,apex_flag=1;
	unsigned char i,j,x,y,m;	
	if(startFlag==0)//preparation work
	{
		recTimeMs=systemTimeMs;
		recApexValue=0;
		temp=0;
	}		
	else
	{	
		if(systemTimeMs<recTimeMs)		 recTimeMs=systemTimeMs;
		if(systemTimeMs-recTimeMs>10)
		{	
			recTimeMs=systemTimeMs;			
			if(apexValue<1)
			{	
				recApexValue=33;
				temp=0;
			}
			else temp=apexValue;					
			// over area (40~60)
			if(recApexValue!=temp)
			{
				if(temp>=30) recApexValue=30;
				if(temp==0) recApexValue=temp;
				if(recApexValue>temp) 
				{
					if(recApexValue==1) recApexValue=0;					
					else recApexValue--;
				}
				else if(recApexValue<temp)
				{
					recApexValue++;	
				}
				else 	recApexValue=temp;//fresh			
				x=33;//35;
				y=16;	
				m=x;
				for(i=0;i<3;i++)
				{		
					x=m+i*5;		
					if(recApexValue==0)
					{
	//				disp_DrawColumn(x,y,40,0);
						disp_DrawColumn(x+1,y+1,38,1);
						disp_DrawColumn(x+2,y,40,1);
						disp_DrawColumn(x+3,y,40,1);
						disp_DrawColumn(x+4,y+1,38,1);
					}
					else if(recApexValue>25)
					{
						disp_DrawColumn(x,y,40,0);
						disp_DrawColumn(x+1,y+1,38,0);
						disp_DrawColumn(x+2,y,40,0);
						disp_DrawColumn(x+3,y,40,0);
						disp_DrawColumn(x+4,y+1,38,0);
					}
					else
					{	
						j=(i<recApexValue)?0:1;					
	//				disp_DrawColumn(x,y,40,0);
						disp_DrawColumn(x+1,y+1,38,j);
						disp_DrawColumn(x+2,y,40,j);
						disp_DrawColumn(x+3,y,40,j);
						disp_DrawColumn(x+4,y+1,38,j);
					}	
				}	
				// apex  area (60~100)
				x=50;//52;
				y=16;
				m=x;
				for(i=0;i<7;i++)
				{
					x=m+i*6;
					if(recApexValue==0)
					{
	//				disp_DrawColumn(x,y,40,0);
						disp_DrawColumn(x+1,y+2,36,1);
						disp_DrawColumn(x+2,y+1,38,1);
						disp_DrawColumn(x+3,y,40,1);
						disp_DrawColumn(x+4,y+1,38,1);
						disp_DrawColumn(x+5,y+2,36,1);
					}
					else if(recApexValue>25)
					{
	//				disp_DrawColumn(x,y,40,0);
						if(i==sys_param_un.device_param.ref_tine)
						{
							disp_DrawColumn(x+1,y+2,36,1);
							disp_DrawColumn(x+2,y+1,38,1);
							disp_DrawColumn(x+3,y,40,1);
							disp_DrawColumn(x+4,y+1,38,1);
							disp_DrawColumn(x+5,y+2,36,1);						
	//						disp_DrawRow(x+3,y+41,1,1);
	//						disp_DrawRow(x+2,y+42,3,1);
	//						disp_DrawRow(x+1,y+43,5,1);
						}
						else
						{						
							disp_DrawColumn(x+1,y+2,36,0);
							disp_DrawColumn(x+2,y+1,38,0);
							disp_DrawColumn(x+3,y,40,0);
							disp_DrawColumn(x+4,y+1,38,0);
							disp_DrawColumn(x+5,y+2,36,0);						
						}					
					}
					else 
					{
						j=((i+3)<recApexValue)?0:1;
	//					disp_DrawColumn(x,y,40,0);
						disp_DrawColumn(x+1,y+2,36,j);
						disp_DrawColumn(x+2,y+1,38,j);
						disp_DrawColumn(x+3,y,40,j);
						disp_DrawColumn(x+4,y+1,38,j);
						disp_DrawColumn(x+5,y+2,36,j);			
					}		
				}
				// oter area  闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?锋俊鎼佸煛閸屾矮绨介梻浣呵归張�?�傜矙閹达富鏁傞柨鐕傛�??100~160)
				x=94;//96;
				y=16;
				m=x;
				for(i=0;i<15;i++)
				{
					x=m+i*4;
					if(recApexValue==0)
					{
	//				disp_DrawColumn(x,y,20,0);				
						if(i<4)
						{
							disp_DrawColumn(x+1,y+1,18+20-i*6,1);
							disp_DrawColumn(x+2,y,20+20-i*6,1);
							disp_DrawColumn(x+3,y+1,18+20-i*6,1);
						}
						else
						{
							disp_DrawColumn(x+1,y+1,17,1);
							disp_DrawColumn(x+2,y,19,1);
							disp_DrawColumn(x+3,y+1,17,1);
						}					
					}
					else if(recApexValue>25)
					{
	//				disp_DrawColumn(x,y,20,0);
					if(i+7==sys_param_un.device_param.ref_tine)//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?節瀹曞爼鍩℃担鍦簴缂傚倷鑳舵繛鈧紒鐘崇墵瀵鈽夐�?�鐘栥劍銇�?�?鍌氬付�?�ゎ偀鏅濈槐鎾存�?�閹绘帊澹曢柣搴″帨閸�??捇鏌涢弴銊ュ�?闁告ɑ鎹囧铏光偓鍦У閵嗗啰绱掗埀顒佺瑹閳ь剙�?�ｉ崘娴�?瀻闁规�?�鏅欑花濠氭椤愩垺澶勯柟鎼佺畺瀹曟椽鏁愰崱�?哄伎婵犵數濮�?幊蹇涱敂閻�?�粯鐓欓柣鐔告緲椤忣厼鈹戦埄鍐╁�?�?┑锛�?厴閺佸倿鏌ㄩ姘婵°倧绲介崯顖炲煕閹达附鐓曟繝闈涙椤忔挳鏌涙惔锛勭闁靛洤瀚伴、�?�€�?╅幓鎺戠闂備胶�?笟妤呭窗濞戞氨涓嶆繛鎴炃氬Σ�?熺�?�閸℃�?�?堟い蹇曞枛濮婄粯鎷呴懞銉с�?婇梺闈╃秶缁犳捇鐛箛娑欐�?�闁跨噦鎷�
					{
						if(i<4)
						{
							disp_DrawColumn(x+1,y+1,18+20-i*6,1);
							disp_DrawColumn(x+2,y,20+20-i*6,1);
							disp_DrawColumn(x+3,y+1,18+20-i*6,1);
						}
						else
						{
							disp_DrawColumn(x+1,y+1,17,1);
							disp_DrawColumn(x+2,y,19,1);
							disp_DrawColumn(x+3,y+1,17,1);
						}	
					}
					else 
					{
						if(i<4)
						{
							disp_DrawColumn(x+1,y+1,18+20-i*6,0);
							disp_DrawColumn(x+2,y,20+20-i*6,0);
							disp_DrawColumn(x+3,y+1,18+20-i*6,0);
						}
						else
						{
							disp_DrawColumn(x+1,y+1,17,0);
							disp_DrawColumn(x+2,y,19,0);
							disp_DrawColumn(x+3,y+1,17,0);
						}									
					}						
				}
				else
				{		
					j=((i+11)<recApexValue)?0:1;					
					//disp_DrawColumn(x,y,20,0);
					if(i<4)
					{
						disp_DrawColumn(x+1,y+1,18+20-i*6,j);
						disp_DrawColumn(x+2,y,20+20-i*6,j);
						disp_DrawColumn(x+3,y+1,18+20-i*6,j);
					}
					else
					{
						disp_DrawColumn(x+1,y+1,17,j);
						disp_DrawColumn(x+2,y,19,j);
						disp_DrawColumn(x+3,y+1,17,j);
					}					
				}	
			}				
			if(apexValue<0)//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵炩拃鍥х閺�?�牆澧界壕鍨归�?鈧崹璺�?潖閾忓湱纾兼俊�?�濆吹閸欏�?��?�虹涵鍛彧闁挎碍銇�?銏㈢闁圭厧婀遍幉鎾礋椤愩倧绱￠梻鍌欑窔濞佳勭仚闂佺ǹ瀛╅悡锟犲箖濡や降鍋呴柛鎰ㄦ杹閹锋椽姊洪崨濠�?畵閻庢氨鍏樺畷鏇＄疀閺傝绨诲銈嗘尵婵挳宕㈤幘�?�界厽婵炴垵宕弸锔剧磼閻樺�?鈽�?�柍钘�?�槸閳�?�氦绠涙繝鍌欏婵犵數濮电喊宥�?�偂閻�?�粯鐓欐い鎾跺枎缁�?�帡鏌涢�?鈧划�?�囨崲濞戙垹宸濇い鎰╁灮娴煎牆鈹戦�?烽練婵炲拑缍侀�?鎴﹀礋椤�?鈺�?煏�?�舵稑顩憸鐗堝哺濮婄粯鎷呴悜妯烘畬闂佹悶鍊�?悧鐘荤嵁�?囨稒鏅搁柨鐕傛�??
			{					
//				OLED_ShowString(110,40," -- ",24,1);
			}
			else if(apexValue<3)
			{
				OLED_ShowString(106,40,"OVER",24,1);//110,40,"OVER",24,1);
			}
			else if(apexValue>25)
			{		
				if(apex_flag==1)
				{
					OLED_ShowString(106,40,"APEX",24,1);//(110,40,"APEX",24,1);
					apex_flag=0;
//					APEXIdleTimeManage(xTaskGetTickCount(),0);
				}								
			}
			else
			{			
				if(recApexValue<sys_param_un.device_param.ref_tine+3) m=0;
				else m=recApexValue-sys_param_un.device_param.ref_tine-3;
				OLED_ShowChar(106,40,' ',24,1);//(110,40,' ',24,1);
				if(m<10)
				{
					if(recApexValue<sys_param_un.device_param.ref_tine+3)
					{
						OLED_ShowChar(118,40,'-',24,1);//(122,40,'-',24,1);
						OLED_ShowChar(130,40,'-',24,1);//(134,40,'-',24,1);				
					}
					else 
					{
						OLED_ShowChar(118,40,' ',24,1);//(122,40,' ',24,1);
						OLED_ShowNum(130,40,m,1,24,1);//(134,40,m,1,24,1);
					}					
				}
				else
				{
					OLED_ShowNum(118,40,m,2,24,1);//(122,40,m,2,24,1);
				}				
				OLED_ShowChar(142,40,' ',24,1);//(146,40,' ',24,1);
			}
			xSemaphoreGive(xSemaphoreDispRfresh);			
		}			
	    blinkTimeMs++;		   
		if(blinkTimeMs>50)//500ms
		{
			blinkTimeMs=0;
			blink=(blink)?0:1;		
			if(apexValue<0)
			{
				if(blink==0)	OLED_ShowString(106,40,"    ",24,1);//(110,40,"    ",24,1);
				else OLED_ShowString(106,40,"APEX",24,1);//(110,40,"APEX",24,1);
			}
			else
			{
				if(recApexValue<26&&recApexValue!=sys_param_un.device_param.ref_tine+3)
				{
					if(sys_param_un.device_param.ref_tine<7)
					{
						x=50+sys_param_un.device_param.ref_tine*6;//52+sys_param_un.device_param.ref_tine*6;
						y=16;
						disp_DrawColumn(x+1,y+2,36,blink);
						disp_DrawColumn(x+2,y+1,38,blink);
						disp_DrawColumn(x+3,y,40,blink);
						disp_DrawColumn(x+4,y+1,38,blink);
						disp_DrawColumn(x+5,y+2,36,blink);
					}
					else if(sys_param_un.device_param.ref_tine<11)
					{
//						x=96+(sys_param_un.device_param.ref_tine-7)*4;
						x=68+sys_param_un.device_param.ref_tine*4-2;
						y=16;	
						disp_DrawColumn(x+1,y+1,80-sys_param_un.device_param.ref_tine*6,blink);
						disp_DrawColumn(x+2,y,82-sys_param_un.device_param.ref_tine*6,blink);
						disp_DrawColumn(x+3,y+1,80-sys_param_un.device_param.ref_tine*6,blink);
					}
					else 
					{							
						x=68+sys_param_un.device_param.ref_tine*4-2;
						y=16;		
						disp_DrawColumn(x+1,y+1,17,blink);
						disp_DrawColumn(x+2,y,19,blink);
						disp_DrawColumn(x+3,y+1,17,blink);
					}
				}
				
			}
		}
	}
	}
	
}
static void ApexBarFlashForSet(unsigned int systemTimeMs,unsigned char startFlag, short int apexValue)
{
	static unsigned int recTimeMs;
	static unsigned short recApexValue,temp;
	unsigned char i,x,y,m;	
	if(startFlag==0)//preparation work
	{
		recTimeMs=systemTimeMs;
		recApexValue=0;
		temp=0;
	}		
	else
	{	
		if(systemTimeMs<recTimeMs)		 recTimeMs=systemTimeMs;
		if(systemTimeMs-recTimeMs>10)
		{	
			recTimeMs=systemTimeMs;			
			if(apexValue<1)
			{			
//				if(temp!=0)
					recApexValue=33;
					temp=0;
			}
			else temp=apexValue;					
			// over area (40~60)
			if(recApexValue!=temp)
			{
				if(temp>=30) recApexValue=30;//
				if(temp==0) recApexValue=temp;//闂傚倸鍊搁崐鎼佸磹閻戣姤鍤勯柤鍝ユ暩娴犳氨绱撻崒娆愮グ�?�ゆ泦鍥ㄥ亱闁�?�儳纾弳锔界節闂堟侗鍎忕紒鐙呯�?閺屻劑�??村Δ鈧�?�楣冩⒑閸濆嫷鍎庣紒鑸靛哺瀵鎮㈤悡搴ｎ唹闂侀�?涘嵆濞佳冣枔椤撶偐鏀介柍钘�?�娴滄繈鏌ㄩ弴�?虹伈鐎�?�喖�?峰鎾閻樿鏁�?�繝鐢靛█濞佳兠归崒姣兼�?�?欏顔藉瘜闂侀潧鐗嗛幊姗€�?板⿰�?熺厵闁告�?鍊栫�?氾拷
				if(recApexValue>temp) 
				{
					if(recApexValue==1) recApexValue=0;					
					else recApexValue--;
				}
				else if(recApexValue<temp)
				{
					recApexValue++;	
				}
				else 	recApexValue=temp;//fresh			
				x=33-16;//35;
				y=16;	
				m=x;
				for(i=0;i<3;i++)
				{		
					x=m+i*5;	
					disp_DrawColumn(x,y,40,0);
					disp_DrawColumn(x+1,y+1,38,0);
					disp_DrawColumn(x+2,y,40,0);
					disp_DrawColumn(x+3,y,40,0);
					disp_DrawColumn(x+4,y+1,38,0);
				}	
				// apex  area (60~100)
				x=50-16;//52;
				y=16;
				m=x;
				for(i=0;i<7;i++)
				{
					x=m+i*6;
//				disp_DrawColumn(x,y,40,0);
					if(i==sys_param_un.device_param.ref_tine)//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?節瀹曞爼鍩℃担鍦簴缂傚倷鑳舵繛鈧紒鐘崇墵瀵鈽夐�?�鐘栥劍銇�?�?鍌氬付�?�ゎ偀鏅濈槐鎾存�?�閹绘帊澹曢柣搴″帨閸�??捇鏌涢弴銊ュ�?闁告ɑ鎹囧铏光偓鍦У閵嗗啰绱掗埀顒佺瑹閳ь剙�?�ｉ崘娴�?瀻闁规�?�鏅欑花濠氭椤愩垺澶勯柟鎼佺畺瀹曟椽鏁愰崱�?哄伎婵犵數濮�?幊蹇涱敂閻�?�粯鐓欓柣鐔告緲椤忣厼鈹戦埄鍐╁�?�?┑锛�?厴閺佸倿鏌ㄩ姘婵°倧绲介崯顖炲煕閹达附鐓曟繝闈涙椤忔挳鏌涙惔锛勭闁靛洤瀚伴、�?�€�?╅幓鎺戠闂備胶�?笟妤呭窗濞戞氨涓嶆繛鎴炃氬Σ�?熺�?�閸℃�?�?堟い蹇曞枛濮婄粯鎷呴懞銉с�?婇梺闈╃秶缁犳捇鐛箛娑欐�?�闁跨噦鎷�
					{
						disp_DrawColumn(x+1,y+2,36,1);
						disp_DrawColumn(x+2,y+1,38,1);
						disp_DrawColumn(x+3,y,40,1);
						disp_DrawColumn(x+4,y+1,38,1);
						disp_DrawColumn(x+5,y+2,36,1);						
//						disp_DrawRow(x+3,y+41,1,1);
//						disp_DrawRow(x+2,y+42,3,1);
//						disp_DrawRow(x+1,y+43,5,1);
					}
					else
					{						
						disp_DrawColumn(x+1,y+2,36,0);
						disp_DrawColumn(x+2,y+1,38,0);
						disp_DrawColumn(x+3,y,40,0);
						disp_DrawColumn(x+4,y+1,38,0);
						disp_DrawColumn(x+5,y+2,36,0);						
					}	
				}
				// oter area  
				x=94-16;//96;
				y=16;
				m=x;
				for(i=0;i<15;i++)
				{
					x=m+i*4;
	//				disp_DrawColumn(x,y,20,0);
					if(i+7==sys_param_un.device_param.ref_tine)
					{
						if(i<4)
						{
							disp_DrawColumn(x+1,y+1,18+20-i*6,1);
							disp_DrawColumn(x+2,y,20+20-i*6,1);
							disp_DrawColumn(x+3,y+1,18+20-i*6,1);
						}
						else
						{
							disp_DrawColumn(x+1,y+1,17,1);
							disp_DrawColumn(x+2,y,19,1);
							disp_DrawColumn(x+3,y+1,17,1);
						}	
					}
					else 
					{
						if(i<4)
						{
							disp_DrawColumn(x+1,y+1,18+20-i*6,0);
							disp_DrawColumn(x+2,y,20+20-i*6,0);
							disp_DrawColumn(x+3,y+1,18+20-i*6,0);
						}
						else
						{
							disp_DrawColumn(x+1,y+1,17,0);
							disp_DrawColumn(x+2,y,19,0);
							disp_DrawColumn(x+3,y+1,17,0);
						}									
					}			
				}			
				OLED_ShowString(92,40,"APEX",24,1);//(110,40,"APEX",24,1);
				xSemaphoreGive(xSemaphoreDispRfresh);
			}			
		}
	}	
}
/**
  * @brief 	disp_ApexPrepare
  * @param   u8 x,u8 y,start coord 
  * @retval none
  */
static void Disp_ApexPreForSet(u8 value)
{
	u8 i,x,y;
//bat area  
	//P0 ,bat	
	// over area (40~60)
	x=34-16;//36;//40-4;
	y=8;	
	for(i=0;i<2;i++)
	{		
		disp_DrawRow(x,y+i+3,15,1);
	}		
	// apex  area (60~102)
		x=52-16;//54;//60-6;
		y=8;
		for(i=0;i<2;i++)
		{
			OLED_DrawPoint(x, y+i+1,1);
			OLED_DrawPoint(x+1, y+i+1,1);
			disp_DrawRow(x,y+i+3,40,1);
		}			
		OLED_ShowNum(x-2,0,0,1,8,1);
		//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幍濡炪倖�?��?�崢褍危婵犳碍鐓�?悹鍥ㄧ叀閸欏�?顨ラ悙鍙夊枠妞ゃ垺顨婂畷鎺戔�?閸曨偅鐝梻鍌氬�?搁崐鐑芥倿閿曞倸绠�?柛�?�ｆ礀绾惧綊鏌″搴″�?�闁稿顑�?�弻锝呂熼崹顔兼�?閻庤鎸风�?��?�舵偂椤愶箑鐐婇柕濞р偓婵洭姊虹粙鍖″伐妞ゎ厾鍏樺濠氬Χ婢跺﹦鐣抽梺鍦劋閸ㄥ灚鎱ㄩ弴鐐╂斀闁绘劕�??堕崳鐑芥煕閵娿劍顥�?�い顐㈢箰鐓ゆい蹇撳缁卞爼姊洪崨濠�?闁稿鍋撻梺绋款儐閹瑰洤鐣锋總绋垮嵆闁绘劗顣槐�?�€姊绘担鍛婃儓缂佸绶�?畷鎴﹀焵椤掑�??鐓曢悗锝庡亝鐏忣厽銇�?锝囩疄闁�?�喗鐟╅、妤�?焵椤掑�?�鏁傞柨鐕傛�?
//		sys_param_un.device_param.ref_tine=(sys_param_un.device_param.ref_tine%7);//offset	max=3	
//		disp_DrawRow(x+1+sys_param_un.device_param.ref_tine*6,60,1,1);//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幍濡炪倖�?��?�崢褍危婵犳碍鐓�?悹鍥ㄧ叀閸欏�?顨ラ悙鍙夊枠妞ゃ垺顨婂畷鎺戔�?閸曨偅鐝梻鍌氬�?搁崐鐑芥倿閿曞倸绠�?柛�?�ｆ礀绾惧綊鏌″搴″�?�闁稿顑�?�弻锝呂熼崹顔兼�?閻庤鎸风�?��?�舵偂椤愶箑鐐婇柕濞р偓婵洭姊虹粙鍖″伐妞ゎ厾鍏樺濠氬Χ婢跺﹦鐣抽梺鍦劋閸ㄥ灚鎱ㄩ弴鐐╂斀闁绘劕�??堕崳鐑芥煕閵娿劍顥�?�い顐㈢箰鐓ゆい蹇撳缁卞爼姊洪崨濠�?闁稿鍋撻梺绋款儐閹瑰洤鐣锋總绋垮嵆闁绘劗顣槐�?�€姊绘担鍛婃儓缂佸绶�?畷鎴﹀焵椤掑�??鐓曢悗锝庡亝鐏忣厽銇�?锝囩疄闁�?�喗鐟╅、妤�?焵椤掑�?�鏁傞柨鐕傛�?
//		disp_DrawRow(x+sys_param_un.device_param.ref_tine*6,61,3,1);
//		disp_DrawRow(x-1+sys_param_un.device_param.ref_tine*6,62,5,1);			
		sys_param_un.device_param.ref_tine=(sys_param_un.device_param.ref_tine%13);//offset	max=3	
//		
//		disp_DrawRow(x-1,60,59,0);//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幍濡炪倖�?��?�崢褍危婵犳碍鐓�?悹鍥ㄧ叀閸欏�?顨ラ悙鍙夊枠妞ゃ垺顨婂畷鎺戔�?閸曨偅鐝梻鍌氬�?搁崐鐑芥倿閿曞倸绠�?柛�?�ｆ礀绾惧綊鏌″搴″�?�闁稿顑�?�弻锝呂熼崹顔兼�?閻庤鎸风�?��?�舵偂椤愶箑鐐婇柕濞р偓婵洭姊虹粙鍖″伐妞ゎ厾鍏樺濠氬Χ婢跺﹦鐣抽梺鍦劋閸ㄥ灚鎱ㄩ弴鐐╂斀闁绘劕�??堕崳鐑芥煕閵娿劍顥�?�い顐㈢箰鐓ゆい蹇撳缁卞爼姊洪崨濠�?闁稿鍋撻梺绋款儐閹瑰洤鐣锋總绋垮嵆闁绘劗顣槐�?�€姊绘担鍛婃儓缂佸绶�?畷鎴﹀焵椤掑�??鐓曢悗锝庡亝鐏忣厽銇�?锝囩疄闁�?�喗鐟╅、妤�?焵椤掑�?�鏁傞柨鐕傛�?
//		disp_DrawRow(x-1,61,59,0);
//		disp_DrawRow(x-1,62,59,0);	
		
		disp_DrawRow(x+1+3*6,60,1,1);//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幍濡炪倖�?��?�崢褍危婵犳碍鐓�?悹鍥ㄧ叀閸欏�?顨ラ悙鍙夊枠妞ゃ垺顨婂畷鎺戔�?閸曨偅鐝梻鍌氬�?搁崐鐑芥倿閿曞倸绠�?柛�?�ｆ礀绾惧綊鏌″搴″�?�闁稿顑�?�弻锝呂熼崹顔兼�?閻庤鎸风�?��?�舵偂椤愶箑鐐婇柕濞р偓婵洭姊虹粙鍖″伐妞ゎ厾鍏樺濠氬Χ婢跺﹦鐣抽梺鍦劋閸ㄥ灚鎱ㄩ弴鐐╂斀闁绘劕�??堕崳鐑芥煕閵娿劍顥�?�い顐㈢箰鐓ゆい蹇撳缁卞爼姊洪崨濠�?闁稿鍋撻梺绋款儐閹瑰洤鐣锋總绋垮嵆闁绘劗顣槐�?�€姊绘担鍛婃儓缂佸绶�?畷鎴﹀焵椤掑�??鐓曢悗锝庡亝鐏忣厽銇�?锝囩疄闁�?�喗鐟╅、妤�?焵椤掑�?�鏁傞柨鐕傛�?
		disp_DrawRow(x+3*6,61,3,1);
		disp_DrawRow(x-1+3*6,62,5,1);		
	// other area 闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?锋俊鎼佸煛閸屾矮绨介梻浣呵归張�?�傜矙閹达富鏁傞柨鐕傛�??100~160闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?锋俊鎼佸煛閸屾矮绨介梻浣呵归張�?�傜矙閹达富鏁傞柨鐕傛�??
		x=95-16;//97;//100-3;
		y=8;
		for(i=0;i<2;i++)
		{
		  OLED_DrawPoint(x, y+i+1,1);//1
			OLED_DrawPoint(x+1, y+i+1,1);
			
			OLED_DrawPoint(x+25, y+i+1,1);
			OLED_DrawPoint(x+1+25, y+i+1,1);
			
			OLED_DrawPoint(x+53, y+i+1,1);
			OLED_DrawPoint(x+1+53, y+i+1,1);
			
			disp_DrawRow(x,y+i+3,60,1);
		}		
		//num
		OLED_ShowNum(x-2,0,1,1,8,1);
		OLED_ShowNum(x+28-5,0,2,1,8,1);
		OLED_ShowNum(x+56-5,0,3,1,8,1);
}
/**
  * @brief 	disp_ApexPrepare
  * @param   u8 x,u8 y,start coord 
  * @retval none
  */
static void Disp_ApexPrepare(u8 value)
{
	u8 i,x,y;
//bat area  闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?锋俊鎼佸煛閸屾矮绨介梻浣呵归張�?�傜矙閹达富鏁傞柨鐕傛�??0~40闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?锋俊鎼佸煛閸屾矮绨介梻浣呵归張�?�傜矙閹达富鏁傞柨鐕傛�??
	//P0 ,bat
	
	// over area (40~60)
	x=34;//36;//40-4;
	y=8;	
	for(i=0;i<2;i++)
	{		
		disp_DrawRow(x,y+i+3,15,1);
	}		
	// apex  area (60~102)
		x=52;//54;//60-6;
		y=8;
		for(i=0;i<2;i++)
		{
			OLED_DrawPoint(x, y+i+1,1);
			OLED_DrawPoint(x+1, y+i+1,1);
			disp_DrawRow(x,y+i+3,40,1);
		}			
		OLED_ShowNum(x-2,0,0,1,8,1);
		//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幍濡炪倖�?��?�崢褍危婵犳碍鐓�?悹鍥ㄧ叀閸欏�?顨ラ悙鍙夊枠妞ゃ垺顨婂畷鎺戔�?閸曨偅鐝梻鍌氬�?搁崐鐑芥倿閿曞倸绠�?柛�?�ｆ礀绾惧綊鏌″搴″�?�闁稿顑�?�弻锝呂熼崹顔兼�?閻庤鎸风�?��?�舵偂椤愶箑鐐婇柕濞р偓婵洭姊虹粙鍖″伐妞ゎ厾鍏樺濠氬Χ婢跺﹦鐣抽梺鍦劋閸ㄥ灚鎱ㄩ弴鐐╂斀闁绘劕�??堕崳鐑芥煕閵娿劍顥�?�い顐㈢箰鐓ゆい蹇撳缁卞爼姊洪崨濠�?闁稿鍋撻梺绋款儐閹瑰洤鐣锋總绋垮嵆闁绘劗顣槐�?�€姊绘担鍛婃儓缂佸绶�?畷鎴﹀焵椤掑�??鐓曢悗锝庡亝鐏忣厽銇�?锝囩疄闁�?�喗鐟╅、妤�?焵椤掑�?�鏁傞柨鐕傛�?
//		sys_param_un.device_param.ref_tine=(sys_param_un.device_param.ref_tine%7);//offset	max=3	
//		disp_DrawRow(x+1+sys_param_un.device_param.ref_tine*6,60,1,1);//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幍濡炪倖�?��?�崢褍危婵犳碍鐓�?悹鍥ㄧ叀閸欏�?顨ラ悙鍙夊枠妞ゃ垺顨婂畷鎺戔�?閸曨偅鐝梻鍌氬�?搁崐鐑芥倿閿曞倸绠�?柛�?�ｆ礀绾惧綊鏌″搴″�?�闁稿顑�?�弻锝呂熼崹顔兼�?閻庤鎸风�?��?�舵偂椤愶箑鐐婇柕濞р偓婵洭姊虹粙鍖″伐妞ゎ厾鍏樺濠氬Χ婢跺﹦鐣抽梺鍦劋閸ㄥ灚鎱ㄩ弴鐐╂斀闁绘劕�??堕崳鐑芥煕閵娿劍顥�?�い顐㈢箰鐓ゆい蹇撳缁卞爼姊洪崨濠�?闁稿鍋撻梺绋款儐閹瑰洤鐣锋總绋垮嵆闁绘劗顣槐�?�€姊绘担鍛婃儓缂佸绶�?畷鎴﹀焵椤掑�??鐓曢悗锝庡亝鐏忣厽銇�?锝囩疄闁�?�喗鐟╅、妤�?焵椤掑�?�鏁傞柨鐕傛�?
//		disp_DrawRow(x+sys_param_un.device_param.ref_tine*6,61,3,1);
//		disp_DrawRow(x-1+sys_param_un.device_param.ref_tine*6,62,5,1);			
		sys_param_un.device_param.ref_tine=(sys_param_un.device_param.ref_tine%13);//offset	max=3	
//		
//		disp_DrawRow(x-1,60,59,0);//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幍濡炪倖�?��?�崢褍危婵犳碍鐓�?悹鍥ㄧ叀閸欏�?顨ラ悙鍙夊枠妞ゃ垺顨婂畷鎺戔�?閸曨偅鐝梻鍌氬�?搁崐鐑芥倿閿曞倸绠�?柛�?�ｆ礀绾惧綊鏌″搴″�?�闁稿顑�?�弻锝呂熼崹顔兼�?閻庤鎸风�?��?�舵偂椤愶箑鐐婇柕濞р偓婵洭姊虹粙鍖″伐妞ゎ厾鍏樺濠氬Χ婢跺﹦鐣抽梺鍦劋閸ㄥ灚鎱ㄩ弴鐐╂斀闁绘劕�??堕崳鐑芥煕閵娿劍顥�?�い顐㈢箰鐓ゆい蹇撳缁卞爼姊洪崨濠�?闁稿鍋撻梺绋款儐閹瑰洤鐣锋總绋垮嵆闁绘劗顣槐�?�€姊绘担鍛婃儓缂佸绶�?畷鎴﹀焵椤掑�??鐓曢悗锝庡亝鐏忣厽銇�?锝囩疄闁�?�喗鐟╅、妤�?焵椤掑�?�鏁傞柨鐕傛�?
//		disp_DrawRow(x-1,61,59,0);
//		disp_DrawRow(x-1,62,59,0);	
		
		disp_DrawRow(x+1+3*6,60,1,1);//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幍濡炪倖�?��?�崢褍危婵犳碍鐓�?悹鍥ㄧ叀閸欏�?顨ラ悙鍙夊枠妞ゃ垺顨婂畷鎺戔�?閸曨偅鐝梻鍌氬�?搁崐鐑芥倿閿曞倸绠�?柛�?�ｆ礀绾惧綊鏌″搴″�?�闁稿顑�?�弻锝呂熼崹顔兼�?閻庤鎸风�?��?�舵偂椤愶箑鐐婇柕濞р偓婵洭姊虹粙鍖″伐妞ゎ厾鍏樺濠氬Χ婢跺﹦鐣抽梺鍦劋閸ㄥ灚鎱ㄩ弴鐐╂斀闁绘劕�??堕崳鐑芥煕閵娿劍顥�?�い顐㈢箰鐓ゆい蹇撳缁卞爼姊洪崨濠�?闁稿鍋撻梺绋款儐閹瑰洤鐣锋總绋垮嵆闁绘劗顣槐�?�€姊绘担鍛婃儓缂佸绶�?畷鎴﹀焵椤掑�??鐓曢悗锝庡亝鐏忣厽銇�?锝囩疄闁�?�喗鐟╅、妤�?焵椤掑�?�鏁傞柨鐕傛�?
		disp_DrawRow(x+3*6,61,3,1);
		disp_DrawRow(x-1+3*6,62,5,1);		
	// other area 闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?锋俊鎼佸煛閸屾矮绨介梻浣呵归張�?�傜矙閹达富鏁傞柨鐕傛�??100~160闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?锋俊鎼佸煛閸屾矮绨介梻浣呵归張�?�傜矙閹达富鏁傞柨鐕傛�??
		x=95;//97;//100-3;
		y=8;
		for(i=0;i<2;i++)
		{
		  OLED_DrawPoint(x, y+i+1,1);//1
			OLED_DrawPoint(x+1, y+i+1,1);
			
			OLED_DrawPoint(x+25, y+i+1,1);
			OLED_DrawPoint(x+1+25, y+i+1,1);
			
			OLED_DrawPoint(x+53, y+i+1,1);
			OLED_DrawPoint(x+1+53, y+i+1,1);
			
			disp_DrawRow(x,y+i+3,60,1);
		}		
		//num
		OLED_ShowNum(x-2,0,1,1,8,1);
		OLED_ShowNum(x+28-5,0,2,1,8,1);
		OLED_ShowNum(x+56-5,0,3,1,8,1);
}

/**
  * @brief 	FreshTorquePictureInApexAndMotorMode
  * @param   u8 x,u8 y,start coord 
  * @retval none
  */
static void FreshTorquePictureInApexAndMotorMode(u8 x,u8 y,unsigned short int torqueValue,unsigned  int systemTimeMs)
{
	static unsigned short int temp;
	static unsigned  int recTimeMs;
	static unsigned  char blinkFlag=1;
	u8 i;
	if(systemTimeMs<recTimeMs) recTimeMs=systemTimeMs;
	if(systemTimeMs-recTimeMs>500)
	{
		recTimeMs=systemTimeMs;	
		blinkFlag =(blinkFlag)?0:1;
		if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)
		{
			temp=(torqueValue*6/torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum]);
		}
		else
		{
			temp=(torqueValue*6/torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]);
		}			
//		temp=(torqueValue*6/torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]);
		if(temp>6) temp=6;
		for(i=0;i<26;i++)//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐闁�?�繒鍋犻�?�鎱ㄩ崒鐐�?�厵妞ゆ柨鎼埀顒佺箓閻ｇ兘骞掗幋鏃�?�?嶅┑顔筋殔濡梻妲愰敓锟�?
		{
			disp_DrawRow(x+3,y+3+i,26,0);
		}
		for(i=0;i<2+4*temp;i++)//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐闁�?�繒鍋犻�?�鎱ㄩ崒鐐�?�厵妞ゆ柨鎼埀顒佺箓閻ｇ兘骞掗幋鏃�?�?嶅┑顔筋殔濡梻妲愰敓锟�?
		{
			if(temp>5)
			{
				disp_DrawRow(x+15-2*temp,y+i+15-2*temp,2+4*temp,blinkFlag);
			}
			else
			{
				disp_DrawRow(x+15-2*temp,y+i+15-2*temp,2+4*temp,1);
			}
		}
		xSemaphoreGive(xSemaphoreDispRfresh);
	}	
}
/**
  * @brief 	Disp_TorquePictureInApexAndMotorMode
  * @param   u8 x,u8 y,torqueValue 
  * @retval none
  */
static void Disp_TorquePictureInApexAndMotorMode(u8 x,u8 y,unsigned short int torqueValue)
{
	disp_DrawRow(x,y,32,1);
	disp_DrawRow(x,y+31,32,1);
	disp_DrawColumn(x,y,32,1);
	disp_DrawColumn(x+31,y,32,1);
	
	 //90% temp=1;//10%
//	if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)
//	{
//		temp=(torqueValue*6/torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum]);
//	}
//	else
//	{
//		temp=(torqueValue*6/torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]);
//	}	
//	for(i=0;i<2*temp;i++)
//	{
//		disp_DrawRow(x+3,y+3+i,26,0);
//		disp_DrawRow(x+3,y+2*temp+i,26,0);
//		disp_DrawColumn(x+3+i,y+3,2+4*temp,0);
//		disp_DrawColumn(x+3+2*temp+i,y+3,2+4*temp,0);
//	}
//	for(i=0;i<2+4*temp;i++)
//	{
//		disp_DrawRow(x+15-2*temp,y+i+15-2*temp,2+4*temp,1);
//	}	
}

/**
  * @brief 	disp_MotorAndApexPrepare
  * @param   u8 x,u8 y,start coord 
  * @retval none
  */
static void Disp_MotorAndApexPrepare(unsigned short int value)
{
	uint32_t temp;
//	u8 x,y;
	Disp_TorquePictureInApexAndMotorMode(0,0,0);//torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]);
	if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
	{
		temp=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
	}
	else
	{
		temp=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
	}	
	if((temp)<1000)	
	{
		OLED_ShowChar(0,32,' ',16,1);
		OLED_ShowNum(4,32,temp,3,16,1);
		OLED_ShowChar(28,32,' ',16,1);
	}
	else
	{
		OLED_ShowNum(0,32,temp,4,16,1);					
	}
	OLED_ShowString(4,48,"rpm",16,1);	
	Disp_ApexPrepare(5);
	ApexProgressBarFlash(0,0,0);		
}
/**
  * @brief 	POWER_OFF
  * @param  num
  * @retval none
  */
void OLED_Display_POWER_OFF(unsigned char num)// power off flash
{		
	if(num==0)//low power 
	{
		OLED_ShowString(26,24,"LOW POWER",24,1);	
	}
	else //  power
	{
		OLED_ShowString(26,24,"POWER OFF",24,1);	
	}
}
/**
  * @brief 	work menu
  * @param  workingMode
  * @retval none
  */
void OLED_Display_WORK(unsigned char workingMode,unsigned char startFlag)//pageID (MENU_MOTOR_WORK_PAGE )only motor  (MENU_ONLY_APEX_PAGE) only apex (MENU_APEX_AND_MOTOR_PAGE) motor and apex 
{		
	unsigned short int temp;	
	if(workingMode==MENU_MOTOR_WORK_PAGE)
	{
		OLED_ShowString(0,0,"TQ",24,1);		
		if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
		{
			temp=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
		}
		else temp=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
		if((temp)<1000)	
		{
			OLED_ShowChar(24,32,' ',16,1);
			OLED_ShowNum(0,32,temp,3,16,1);
		}
		else
		{
			OLED_ShowNum(0,32,temp,4,16,1);					
		}
		OLED_ShowString(0,48,"rpm",16,1);	
		//picture
		Disp_TorqueProgressBar(36,0,0);
		if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)
		{
			TorqueProgressBarFlash(0,0,torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum]);
		}
		else if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
		{
			TorqueProgressBarFlash(0,0,torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum]);
		}
		else 	TorqueProgressBarFlash(0,0,torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]);
		//target torque
		if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
		{
			Disp_TorqueOnly(120,40,torque_list[MAX_torque_42_Ncm]);
		}
		else if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)
		{
			Disp_TorqueOnly(120,40,torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum]);
		}
		else
		{
			Disp_TorqueOnly(120,40,torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]);
		}	
	}	
	else if(workingMode==MENU_ONLY_APEX_PAGE)
	{	
		OLED_ShowString(6,4,"P0",16,1);
		Disp_ApexPrepare(60);		
		ApexProgressBarFlash(0,0,0);	
	}	
	else if(workingMode==MENU_APEX_AND_MOTOR_PAGE)
	{				
		Disp_MotorAndApexPrepare(60);
	}		
}
/**
  * @brief 	GC_ControlMotor
  * @param  systemTime
  * @retval none
  */
static void  GC_ControlMotor(int depth,unsigned int systemTimeMs)
{	
	static 	unsigned char auto_flag_ap=null_signal;
	static  unsigned int recTimeMs;
	static eEndoMode  motor_run_mode=Max_endoMode;
	unsigned char sendSignal=null_signal;
	if(recTimeMs>systemTimeMs) recTimeMs=systemTimeMs;
	if(sys_param_un.device_param.apexFunctionLoad==1)
	{
		if(sys_param_un.device_param.use_p_num<10)//
		{
			#ifdef  APEX_FUNCTION_EBABLE	// 
			if(depth<27)
		 	{			 
				if(depth<0)
				{	
					if(motor_status.status!=Status_STOP)//auto start ,in
					{	
						if(sys_param_un.device_param.apical_action_flag!=0)
						{		
							App_MotorControl(MOTOR_MODE_STOP);
						}		
					}						
					auto_flag_ap = motor_apex_run_signal;//		
				}
				else if(depth<=3+sys_param_un.device_param.ref_tine)
				{	//
					if(sys_param_un.device_param.apical_action_flag==1)//
					{
						if(motor_run_mode==Max_endoMode)
						{	
							motor_run_mode=	motor_settings.mode;	
							App_MotorControl(MOTOR_MODE_STOP);
							if(motor_run_mode==EndoModePositionToggle)
							{
								if(motor_settings.forward_position > - motor_settings.reverse_position)
								{
									motor_settings.mode=EndoModeSpeedReverse;
								}
								else if(motor_settings.forward_position < - motor_settings.reverse_position)
								{
									motor_settings.mode=EndoModeSpeedForward;
								}
							}
							else  if(motor_run_mode==EndoModeSpeedForward)
							{
								motor_settings.mode=EndoModeSpeedReverse;
							}
							else  if(motor_run_mode==EndoModeSpeedReverse)
							{
								motor_settings.mode=EndoModeSpeedForward;
							}
							App_MotorControl(MOTOR_SETTING_UPDATE);									
							if(motor_status.status==Status_STOP)
							{	
								if(sys_param_un.device_param.auto_start_flag!=0&&auto_flag_ap != motor_apex_run_signal)
								{
									auto_flag_ap= motor_apex_run_signal;	
								}									
								App_MotorControl(MOTOR_MODE_START);
							}		
						}	
					}
					else if(sys_param_un.device_param.apical_action_flag==2)
					{	
						if(motor_run_mode==Max_endoMode)
						{
							motor_run_mode =	motor_settings.mode;
							if(motor_status.status!=Status_STOP)
							{													
								App_MotorControl(MOTOR_MODE_STOP);
							}			
						}	            			 
					}
					if(sys_param_un.device_param.apical_action_flag!=0||sys_param_un.device_param.auto_start_flag!=0||sys_param_un.device_param.auto_stop_flag!=0)//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滄鐐寸墪鑿愭い鎺嗗亾闁逞屽�?閸ㄥ墎绮�?鍫涗汗闁圭儤鎸鹃崢顏呯節閻㈤潧浠ч柛瀣�?閳�?�秹宕ㄩ�?�咃紲闂侀�?炲苯澧柣锝嗙箞瀹曠喖顢楅崒姘瑲闂傚倷绀侀悿鍥ь浖閵娧呯焼濞撴埃鍋撶�?殿喖鍟块埢搴ㄥ箛閳衡偓缁ㄥ�?呴銏″闁�?�悂绠栧畷娲晲閸℃ê�?挎繝鐢靛Т閹冲繘顢旈悩缁�?�厵闁荤喐�?�橀�?�炩攽閳╁啯鍊愬┑锛�?厴閺佸倿宕ｉ妷褎瀵滈梻鍌�?�?峰ù鍥敋瑜忛埀顒佺▓閺�?繈宕版繝鍌ゅ悑濠㈣泛顑呴崜�?�碱渻閵堝�?�澧遍柛瀣洴瀵憡绗熼埀顒勫蓟閻斿吋鐒介柨鏇楀亾闁诲骏缍侀弻锝夊Χ閸屾矮澹曢梻鍌�?�?风粈渚€骞栭锕�?纾圭憸鐗堝俯閺佸�?绻涢崱妯哄妞ゆ洟浜堕弻鐔煎礈瑜忕敮娑㈡煃闁�?绗掗�?�澶�?煥濠靛�?�澧曠悮姘辩磼缂併垹骞栭柣鏍с偢瀵鈽夐�?�鈥充汗闂佸憡鍔栬ぐ鍐綖閹烘鍊�?繛鍫濈仢閺�??�?鏌熺拠�?�纾块柟骞垮灩閳规垿宕遍埡鍌�?厞婵＄偑鍊栫�?鎺�?�磻閸℃稑鑸归柣銏犳啞閳锋帒霉閿濆牊�?�犻悽�?�涚洴閺屾盯濡搁妷銉﹀�?梺�?涚┒閸旀垶淇婇懜闈涚窞濠电姴瀚悡锝夋⒒娴ｅ摜绉烘俊顐㈡健閹偤鏁冮崒娑樹簵濠电偞鍨崹娲偂閺囥垺鍊堕柣鎰閻ｈ姤銇�?箛鎾跺闁绘帒鐏氶妵鍕箳瀹ュ牆鍘℃繝鈷€鍥︽喚闁哄苯绉烽¨渚€鏌涢幘鏉戝摵闁�?�喓鍨介、妤�?礋椤愩値鍞甸梻浣芥硶閸ｏ箓骞忛敓锟�?
					{
						if(motor_status.status!=Status_STOP)
						{
							if(motor_settings.mode==EndoModePositionToggle)
							{
								if(motor_settings.toggle_mode_speed>speed_list[spd200_Rpm_num])
								{
									motor_settings.toggle_mode_speed=speed_list[spd200_Rpm_num];
									App_MotorControl(MOTOR_SETTING_UPDATE);
								}						
							}
							else 
							{
								if(motor_settings.forward_speed>speed_list[spd200_Rpm_num])
								{							
									motor_settings.forward_speed=speed_list[spd200_Rpm_num];
									motor_settings.reverse_speed=-speed_list[spd200_Rpm_num];									 
									App_MotorControl(MOTOR_SETTING_UPDATE);							
								}
							}							
						}	
					}
			    }
				else if(depth<=(7+sys_param_un.device_param.ref_tine))//0.67//250rpm
				{	
					if(motor_status.status!=Status_STOP)
					{
						if(sys_param_un.device_param.apical_action_flag!=0||sys_param_un.device_param.auto_start_flag!=0||sys_param_un.device_param.auto_stop_flag!=0)
						{
							if(motor_settings.mode==EndoModePositionToggle)
							{
								if(motor_settings.toggle_mode_speed>speed_list[spd250_Rpm_num])
								{
									motor_settings.toggle_mode_speed=speed_list[spd250_Rpm_num];
									App_MotorControl(MOTOR_SETTING_UPDATE);
								}						
							}
							else 
							{
								if(motor_settings.forward_speed>speed_list[spd250_Rpm_num])
								{							
									motor_settings.forward_speed=speed_list[spd250_Rpm_num];
									motor_settings.reverse_speed=-speed_list[spd250_Rpm_num];									 
									App_MotorControl(MOTOR_SETTING_UPDATE);							
								}
							}
						}
					}	
				}
				else if(depth<=(8+sys_param_un.device_param.ref_tine))//300rpm
			 	{	
					if(motor_status.status!=Status_STOP)
					{
						if(sys_param_un.device_param.apical_action_flag!=0||sys_param_un.device_param.auto_start_flag!=0||sys_param_un.device_param.auto_stop_flag!=0)
						{
							if(motor_settings.mode==EndoModePositionToggle)
							{
								if(motor_settings.toggle_mode_speed>speed_list[spd300_Rpm_num])
								{
									motor_settings.toggle_mode_speed=speed_list[spd300_Rpm_num];
									App_MotorControl(MOTOR_SETTING_UPDATE);
								}						
							}
							else 
							{
								if(motor_settings.forward_speed>speed_list[spd300_Rpm_num])
								{							
									motor_settings.forward_speed=speed_list[spd300_Rpm_num];
									motor_settings.reverse_speed=-speed_list[spd300_Rpm_num];									 
									App_MotorControl(MOTOR_SETTING_UPDATE);							
								}
							}
						}
					}	
				}
			    else if(depth<=(9+sys_param_un.device_param.ref_tine))
			    {		
					if(motor_run_mode!=Max_endoMode) 
					{ 
						motor_settings.mode=motor_run_mode;		
						motor_run_mode=Max_endoMode; 
						App_MotorControl(MOTOR_SETTING_UPDATE);	
					}						 
					if(motor_status.status!=Status_STOP)
					{
						if(sys_param_un.device_param.apical_action_flag!=0||sys_param_un.device_param.auto_start_flag!=0||sys_param_un.device_param.auto_stop_flag!=0)
						{
							if(motor_settings.mode==EndoModePositionToggle)
							{
								if(motor_settings.toggle_mode_speed>speed_list[spd350_Rpm_num])
								{
									motor_settings.toggle_mode_speed=speed_list[spd350_Rpm_num];
									App_MotorControl(MOTOR_SETTING_UPDATE);
								}						
							}
							else 
							{
								if(motor_settings.forward_speed>speed_list[spd350_Rpm_num])
								{							
									motor_settings.forward_speed=speed_list[spd350_Rpm_num];
									motor_settings.reverse_speed=-speed_list[spd350_Rpm_num];									 
									App_MotorControl(MOTOR_SETTING_UPDATE);							
								}
							}
						}
					}	
				}
				else
				{
					if(motor_run_mode!=Max_endoMode)
					{
						motor_settings.mode=motor_run_mode ;	
						motor_run_mode=Max_endoMode;
						motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
						motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
						motor_settings.reverse_speed=-motor_settings.forward_speed;						
						App_MotorControl(MOTOR_SETTING_UPDATE);
					}				
					if(motor_status.status!=Status_STOP)
					{
						if(sys_param_un.device_param.apical_action_flag!=0||sys_param_un.device_param.auto_start_flag!=0||sys_param_un.device_param.auto_stop_flag!=0)
						{
							if(motor_settings.mode==EndoModePositionToggle)
							{
								if(motor_settings.toggle_mode_speed!=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum])
								{
									motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
									App_MotorControl(MOTOR_SETTING_UPDATE);
								}	
							}
							else 
							{
								if(motor_settings.toggle_mode_speed!=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
								{							
									motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
									motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];									 
									App_MotorControl(MOTOR_SETTING_UPDATE);							
								}
							}
						}
					}	
				}				
				if(sys_param_un.device_param.auto_start_flag!=0)//auto start ,in 
				{
					if(auto_flag_ap!=motor_apex_run_signal)
					{
						auto_flag_ap= motor_apex_run_signal;										
						if(depth>=0)
						{
							if(motor_status.status==Status_STOP)
							{
								App_MotorControl(MOTOR_MODE_START);
							}											
						}
					}
				}						
			}	
			else 
			{
				if(motor_run_mode!=Max_endoMode) 
				{
					motor_settings.mode=motor_run_mode;	
					motor_run_mode=Max_endoMode;	
					motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
					App_MotorControl(MOTOR_SETTING_UPDATE);
				}	 
				if(motor_settings.forward_speed!=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
				{
					motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					App_MotorControl(MOTOR_SETTING_UPDATE);	
				} 
				if(motor_settings.toggle_mode_speed!=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum])
				{
					motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
					App_MotorControl(MOTOR_SETTING_UPDATE);	
				}
				if(auto_flag_ap!=motor_apex_stop_signal)
				{
					if(sys_param_un.device_param.auto_stop_flag!=0)//exit,
					{
						if(motor_status.status!=Status_STOP)
						{
							 App_MotorControl(MOTOR_MODE_STOP);
						}	
						sendSignal=motor_apex_stop_signal;
						xQueueSend(xQueueKeyMessage, &sendSignal, 0);					
					}						
					auto_flag_ap= motor_apex_stop_signal;						
				}
				if(motor_status.status==Status_STOP&&sys_param_un.device_param.apical_action_flag!=0) 
				{
					App_MotorControl(MOTOR_MODE_START);
				}//restart
			}
			if(sys_param_un.device_param.auto_start_flag==0&&sys_param_un.device_param.auto_stop_flag==0&&sys_param_un.device_param.apical_action_flag==0)//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃娆擃敆閳ь剚鏅堕鐐寸厽闁圭儤娲橀ˉ澶愭懚閺嶎厽鐓ユ繝闈涙閸ｅ綊鏌￠崱娆忎�?闁靛洤瀚版俊鐑藉Ψ瑜嶉埅鐢告⒑閸濆�??瀚扮紒澶屾嚀閻ｇ兘顢曢敃鈧粈瀣亜閹邦喖鏋庡ù婊堢畺閺岀喖�?�荤�?�?瓕绐楅梺杞扮閸婂潡�?诲☉銏╂晝闁挎繂�?涢ˇ銊╂⒑閹惰姤鏁遍柛銊ユ健�?�炲啳銇愰幒鎴犵暢闂佸湱鍎ら崹鐢糕€�?崼銉︹拺闁告繂瀚烽崕鎰版煕閵�?�儲鍋ョ�?殿喖�?烽弫宥�?��?�閵娿儰澹曢梺鎸庣箓缁ㄥジ骞夋ィ鍐╃厽闁规儳鍟块幃鎴�?亜閵婏絽鍔﹂柟�?�界懇閹崇娀顢�?�崒銈呮櫔闂傚倷娴�?鏍窗濡ゅ�?鍨濋柟鎹愵嚙缁犵娀鏌ｉ幇�?�佹儓閸烆垶姊洪幐搴⑩拻闁烩剝�?�圭粋鎺曨樄婵﹤顭峰畷鎺戔枎閹�?厽袦闂備礁婀遍埛�?ュ磻婵犲倻鏆﹂柟鐗堟緲闁卞洭鏌￠崶鈺佷户闁挎稒鐟╁娲传閸曨厸鏋嗛梺绋款儌閸�??捇�?�虹粙娆惧剱闁瑰憡鎮傞敐鐐测攽鐎ｅ灚鏅㈤梺绋挎湰缁ㄤ粙濡搁埡鍌�?��?�闂佸憡绻傜�?氼垶锝為敃鍌涚厱闁哄啠鍋撻柛銊ユ健閻涱噣宕橀鍢�?冾熆鐠虹尨鍔熼柨娑欑矊閳规垿�?欓弶鎴犱桓闂佹�?�娲嶉弲婊呪偓闈涖偢閺佹捇鏁撻敓锟�
			{	
				if(motor_status.status!=Status_STOP&&auto_flag_ap!=motor_apex_run_signal)	auto_flag_ap=motor_apex_run_signal;
				if(motor_status.status==Status_STOP&&auto_flag_ap!=motor_apex_stop_signal)	auto_flag_ap=motor_apex_stop_signal;
			}	
			#endif		
		}
		else
		{
			if(motor_status.status!=Status_STOP&&sys_param_un.device_param.use_p_num==10)
			{													
				App_MotorControl(MOTOR_MODE_STOP);
			}	
		}	
    	if(depth<=0)  //
		{
			if(systemTimeMs>100+recTimeMs)
			{
				recTimeMs=systemTimeMs;
				sendSignal=BUZZER_MODE_GC_CONNECT_TEST;
				xQueueSend(xQueueBeepMode, &sendSignal, 0);
			}	
		}
		else if(depth<(3+sys_param_un.device_param.ref_tine))
		{
			if(systemTimeMs>150+recTimeMs)
			{
				recTimeMs=systemTimeMs;
				sendSignal=BUZZER_MODE_GC_OVER;
				xQueueSend(xQueueBeepMode, &sendSignal, 0);
			 }
		}		
		else if(depth==(3+sys_param_un.device_param.ref_tine))
		{
			if(systemTimeMs>400+recTimeMs)
			{
				recTimeMs=systemTimeMs;
				sendSignal=BUZZER_MODE_GC_ZREO_APEX;
				xQueueSend(xQueueBeepMode, &sendSignal, 0);
			 }
		}		
		else if(depth<(16+sys_param_un.device_param.ref_tine))
		{
			if(systemTimeMs>recTimeMs+800)
			{
				recTimeMs=systemTimeMs;
				 sendSignal=BUZZER_MODE_GC_APEX;
				xQueueSend(xQueueBeepMode, &sendSignal, 0);
			 }
		}		
     	if(depth==30)
		{	
			if(auto_flag_ap!=motor_apex_stop_signal)
			{
				auto_flag_ap= motor_apex_stop_signal;//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?節瀹曞爼鍩℃担鍦偓铏�?磽娴ｇ瓔鍤欐俊�?�ｇ懇婵＄敻宕熼�?�鳖啋闁荤姾娅ｅú鍛存晲�?�跺鈧灚鎱ㄥ鍡�?�箺闁汇劍鍨块弻鐔煎�?�閹烘挸鏆堥梺璇″灡濡啴�??幇鏉跨＜婵﹩鍋勯ˉ姘節濞堝灝鏋熼柨鏇�?�櫊瀹曟粌鈽夊杈╃厠濠电偛�?婃�?�婵�?偂濞戙垺鍊堕柣鎰邦杺閸ゆ瑥鈹戦鐓庘偓鍧楀蓟閻旂⒈鏁婇柛婵嗗娴煎牓鎮楃憴鍕８闁告柨绉堕幑銏�?攽鐎ｎ亞顦板銈�?箰濡盯鏁嶉悢鍏尖拻闁�?�本鑹鹃埀顒勵棑缁牊鎷呴搹鍦槸婵犵數濮村▔姘緞閹邦剛顔掗柣鐘叉穿鐏忔瑩藝瑜嶉埞鎴︻敊婵劒绮堕梺绋�?�儐閹搁箖鍩�?椤掍緡鍟忛柛锝庡櫍瀹曟垶绻濋崶鈺佺ウ濠碘�?�鍨伴崥瀣偓�?�哺閺屾稑鈻庤箛锝嗏枔濠�?��?�鍋嗛崰鏍ь潖缂佹鐟归柍褜鍓欓…鍥樄闁�?�啫鍥у耿婵＄偑鍨虹粙鎴ｇ亙闂佸憡绮堥悞锕傚疾濠婂牊鈷戦悹鎭掑�?�濞�?劙鏌熼崙銈嗗�?
				if(sys_param_un.device_param.auto_stop_flag!=0||sys_param_un.device_param.auto_start_flag!=0||sys_param_un.device_param.apical_action_flag!=0)//exit,闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滄鐐寸墪鑿愭い鎺嗗亾闁�?�浚鍣ｉ弻锝�?攽婵犲倻鍘紓浣虹帛閻╊垰鐣烽崡鐐嶇喖鎼归柅娑欏亝婵犵�?绱曢崑鐘伙綖婢跺绠惧┑鐘叉搐缁犳牜鈧懓瀚竟瀣几鎼�?劎鍙撻柛銉ｅ妽閹�??霉閻橀潧甯舵い顏勫暣婵″爼宕ㄩ�?�庡敹闂備胶�?悧婊堝储瑜旈敐鐐剁疀濞戞瑦鍎柣鐔哥懃鐎氼剛澹曢鐐粹拺闂傚牊鑳嗚ぐ鎺戠？闁哄�??鍎查崐闈浳旈敐鍛�?�闁抽攱鍨块弻娑樷攽閸℃浼岄梺绋块缁绘垿濡甸崟�?�ｆ晣闁绘ɑ褰�?�?瀣⒑缂佹ü绶遍柛鐘崇〒缁鈽�?�Ο閿嬵潔濠电偛妫楃换鍡涘磻閹惧绡€婵﹩鍘鹃崢楣冩⒑鐠団€冲�?�閻㈩垱�?″畷婵嗏�?閸曨厾�?�曢梺鍛婁緱閸�??嫰鎮橀崣澶�?弿濠电姴鍟妵婵堚偓瑙勬处閸�?﹤鐣烽悢纰辨晝闁绘�?�娓规竟鏇㈡⒑閸撴彃浜濇繛鍙夌墱婢�?�洝銇愰幒鎾跺幐閻庡箍鍎�?鍛婄閹扮増鐓曢悗锝庡亝鐏忣厽銇勯锝囩疄妞ゃ垺顨婂畷鎺戔攦閻愵亜�?傛慨濠冩そ瀹曨偊宕熼鍛晧闂備�?�鎲″褰掑垂閸ф宓侀柛鎰靛枛椤懘鏌曢崼婵囧櫣缂佹劖绋掔换婵�?偨闂堟刀銏ゆ倵濮橀棿绨芥俊鍙�?��?�瀵挳濮�?閳锯偓閹风粯绻涙潏鍓ф偧闁稿簺鍊濆畷鐢稿焵椤掑�?鈷戦柟鑲╁仜閻忣�?鏌ｉ幘�?��?�亾瀹曞洦娈鹃梺鍝勮閸庨亶�?屽Δ鈧…璺ㄦ崉閾忓湱鍔稿┑鐐存綑鐎�?澘顫忛搹鍦＜婵☆垰娴氭禍�?�嗙亽婵犵數�?�?崑鍡涙倿閽樺褰掓偂鎼达絾鎲奸梺缁樻尪閸庣敻�??婚敐澶婂嵆闁绘劖绁撮崑鎾诲捶椤撴稑浜�?慨妯煎亾鐎氾�?,闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿鏌涘┑鍡楊伂妞ゎ偓绠撻弻娑㈠籍閳ь剟宕归崸妤冨祦闁告劦鍠栭～鍛存煏閸繃鍣�?紒鎰⊕缁绘繈鎮介�?�娴躲垽�?楀闂寸敖婵″弶鍔欏鎾閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒冾潐濞叉﹢銆冮崨杈剧稏婵犲﹤鐗嗛悞�?亜閹哄棗浜惧銈嗘穿缂嶄線銆侀弴銏℃櫇闁逞屽墰缁粯绻濆�?�炰化闂佹悶鍎滈崘�?�堢崜婵＄偑鍊戦崕鑼崲閸繍�?�栨繛�?�簼椤ュ牊绻涢幋鐐跺妞わ絽鎼埞鎴﹀煡閸℃ぞ绨煎銈冨�?�閿曨亪濡存笟鈧�?�€宕煎┑瀣�?闂備礁鎼ú銊╁疮閸ф绠繛�?�簼閻撶喖鏌ｅΟ鍝勫�?闁煎壊浜弻娑㈠棘鐠恒劎鍔悗娈�?枟閹倿鐛€ｎ喗鏅滈柣锝呰�?��?�炴劙�?�虹拠鎻掑毐缂傚秴妫欑粋宥�?�醇閺囩喎浜楅梺绋跨箳閳峰牆鈻撴禒瀣厽闁归偊鍨伴惃铏�?磼閻樺樊鐓奸柡宀�?鍠栭、娆撴偩瀹€鈧悡澶�?⒑閻熸�?�锛嶉柛瀣ㄥ�?栨穱濠囨倻閼恒儲娅嗙紓浣�?☉椤戝�?�骞�?悜鑺モ拻闁�?�本鑹鹃埀顒勵棑缁牊绗熼埀顒勭嵁閺嶎収鏁冮柨鏇楀亾缁�?儳缍婇弻娑㈩敃閿濆�?�顦ョ紒鐐劤缂嶅﹪�?婚垾鎰佸悑閹肩补鈧尙鐖遍梻浣呵归鍡涘箰閹间緤缍栨繝闈涱儛閺佸棝鏌涚仦鍓ф晼闁靛ň鏅滈埛鎴︽煕濠靛棗�?�い�?�畵閺屾盯�?埀顒勫垂閸ф宓侀柛鎰靛枛椤懘鏌曢崼婵囧櫣缂佹劖绋掔换婵�?偨闂堟刀銏ゆ倵濮橀棿绨芥俊鍙�?��?�瀵挳濮�?閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒冾潐濞叉﹢銆冮崨杈剧稏婵犲﹤鐗嗛悞�?亜閹哄棗浜惧銈嗘穿缂嶄線銆侀弴銏℃櫇闁逞屽墰缁粯绻濆�?�炰化闂佹悶鍎烘禍�?�堟儍濞�?亝鐓熼柟鐐綑閻忋儲銇勯鍕�?�濠碘€崇埣瀹曞�?螖閳ь剝顤傞梻鍌欑閹诧繝骞愮拠�?�僵闁靛ň鏅涢悞�?亜閹哄秷鍏岄柛鐔哥叀閺岀喖宕欓妶鍡�?�伓
				{
					xQueueSend(xQueueKeyMessage, &auto_flag_ap, 0);//濠电姷鏁告慨鐑�?�€傞鐐潟闁哄洢鍨圭壕濠�?煙鏉堝墽鐣辩�?鎹愵潐娣囧﹪濡堕崨�?�兼闂佸搫顑勭粈渚�?鍩為幋锔藉亹閻庡湱濮�?ˉ婵�??⒑閸濆�?�鍎庣紒鑸靛哺瀵鎮㈤悡搴ｎ唹闂侀�?涘嵆濞佳冣枔椤撶偐鏀介柍钘�?�娴滄繈鏌ㄩ弴�?虹伈鐎�?�喖�?峰鎾閻樿鏁�?�繝鐢靛█濞佳兠归崒姣兼�?�?欏顔藉瘜闂侀潧鐗嗛幊鎰不娴煎瓨鐓熼柡�?�庡亜鐢埖銇勯銏㈢闁圭厧婀遍幉鎾礋椤愩倧绱￠梻鍌欑窔濞佳勭仚闂佺ǹ瀛╅悡锟犲箖濡や胶绡�?婵﹩鍘兼�?��?�炩攽閻樿宸ユ俊�?�ｎ殜閹偞绂掔�?ｎ偆鍘遍梺鍝�?�?藉▔鏇″€�?梻渚�?鈧偛鑻晶浼存煛娴ｈ鍊愮�?规洏鍨奸ˇ褰掓煕閳瑰灝鐏柟�?�涙婵℃悂濡疯閺夊憡淇婇悙�?�勨偓鏍暜閹烘鍥敊閻愵剦�?�熼梺璺ㄥ櫐閹凤�?
					App_MotorControl(MOTOR_MODE_STOP);
				}					
			}
		}				
	 }
	else//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?節瀹曞爼鍩℃担鍦簴缂傚倷鑳舵繛鈧紒鐘崇墵瀵鈽夐�?�鐘栥劑鏌曡箛濠傚⒉闁绘繃鐗�?�穱濠囨倷妫版繃缍堟繝鐢靛仜閿曨�?鐛崘銊ф殝闁逛絻娅曢悗濠�?椤愩垺澶勯柟灏栨櫆缁傛帡鎮℃惔顔藉瘜闂侀潧鐗嗛幊姗€�?板⿰�?熺厵闁告�?鍊栫�?氾拷
	{		
		if(auto_flag_ap!=motor_apex_stop_signal)
		{
			auto_flag_ap= motor_apex_stop_signal;//闂傚倸鍊搁崐鎼佸磹閻戣姤鍤勯柛鎾�?閸ㄦ繃銇勯弽銊с�?掑ù鐘冲哺�?婄粯鎷呴搹骞库偓濠囨煛閸涱喚�?掗柟顔ㄥ洦鍋愰悹鍥у级濡差剟�?�洪柅鐐茶�?��?�ь垶鏌曢崶�?��?�鐐�?儔閺佹劙宕卞▎�?�佸亾婵犲�??纾藉ù锝囨嚀婵牊淇婇銏犳殻鐎殿喖�?烽崺鍕礃閳轰緡鈧捇鏌ｉ悢鍝ユ噧閻庢凹鍓涙竟鏇㈠�??閼恒儱鈧灚绻涢崼婵堜虎婵炲懏锕㈤弻娑㈠�?�鐠虹儤鐎炬繛锝�?搐閿曨亜鐣锋總绋�?嵆闁绘劘灏埀顒€鐏濋埞鎴炲箠闁�?�﹥娲熼�?鍐晲閸�?煈�?�熼梺鎸庢�?�閸婂綊鍩涢幒妤佺厱閻忕偛澧介幊鍕磼娴ｅ�?顣肩紒缁樼⊕瀵板�?鈧綆鍋嗛ˇ浼存倵鐟欏�??纾�?�柛妤佸▕閻涱噣宕堕鈧痪褔鏌涢…鎴濇灈婵炲牄鍊曢埞鎴︽偐閸偅姣勬繝娈�?枤閺佸骞婂┑瀣�?鐟滃繑绋夊鍡愪簻闁哄稁鍋勬禒锕傛煟閹惧�?鍔﹂柡宀�?�?瀵挳鎮欏ù瀣壕闁革富鍘搁崑鎾愁潩閻愵剙�?��				
		}	
	}	
}
/**
  * @brief 	MotorRunModeAndTorque
  * @param  
  * @retval none
  */
static void MotorRunModeAndTorque(unsigned short int realTorque, unsigned int systemTimeMs)
{
	unsigned short int sendBuff;
	static unsigned short int overLoadCount;
	static unsigned short int overdCount,beep_time_out;
	if(realTorque>MAX_TORQUE_UUPER_THRESHOLD*1.05)//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑嗛崐閿嬨亜閹哄棗浜惧銈呴獜閹凤拷10%(1.10==5.0N,1.08<=4.8)
	{
		overLoadCount++;
		if(overLoadCount>20)//10)//10*10=100ms
		{
			sendBuff=run_button_press_signal;
			App_MotorControl(MOTOR_MODE_STOP);
			MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);//save data	
			xQueueSend(xQueueKeyMessage, &sendBuff, 0);
			sendBuff= BUZZER_MODE_MUTE;		
			xQueueSend(xQueueBeepMode, &sendBuff, 0);	
			beep_time_out=0;			
		}		
	}
	else
	{
		overLoadCount=0;	
		if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)
		{
			if(sys_param_un.device_param.apical_action_flag!=0&& GC_depth_vlaue(0, 1)<=(9+sys_param_un.device_param.ref_tine)&&sys_param_un.device_param.apexFunctionLoad!=0)				
			{
				;								
			}	
			else
			{
				if(realTorque>torque_list[torque40_Ncm])
				{
					if(motor_settings.mode !=EndoModeKeepForward&&motor_settings.mode!=EndoModeKeepReverse)
					{
						overdCount++;
						if(overdCount>35)//10*30=300ms;0.3S,change for test
						{												
							overdCount=0;
							if(motor_settings.forward_position>-motor_settings.reverse_position)	
							{//
								motor_settings.mode=EndoModeKeepReverse;
							}
							else 
							{
								motor_settings.mode=EndoModeKeepForward;
							}
							motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
							motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
							App_MotorControl(MOTOR_MODE_RESTART);
						}
					}					
				}
				else if(realTorque>=torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum])
				{
					if(motor_settings.mode !=EndoModePositionToggle)
					{	
						overdCount++;
						if(overdCount>40)//10*30=300ms;0.3S,change for test
						{												
							overdCount=0;
							motor_settings.mode=EndoModePositionToggle;
							motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
							motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
							motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];								
							motor_settings.upper_threshold=torque_list[torque40_Ncm]*0.10f;
							motor_settings.lower_threshold= motor_settings.upper_threshold*0.6f;//torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum]*0.10;
							motor_settings.forward_position=motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition;
							motor_settings.reverse_position=motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition;
//							App_MotorControl(MOTOR_SETTING_UPDATE);
							App_MotorControl(MOTOR_MODE_RESTART);
						}							
					}	
				}	
				else //(realTorque<torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum])
				{	
					if(motor_settings.mode !=EndoModeSpeedForward)
					{			
						overdCount++;	
						if(overdCount>40)	//10*60=600ms;0.6S,change for test
						{
							overdCount=0;								
							motor_settings.mode=EndoModeSpeedForward;	
							motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
							motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
							App_MotorControl(MOTOR_MODE_RESTART);
						}
					}						
				}	 //beep				
				if(realTorque>torque_list[torque40_Ncm]*0.80)//
				{
					if(realTorque>torque_list[torque40_Ncm])
					{
						if(beep_time_out>10)//0.2S
						{
							beep_time_out=0;
						}	
					}
					else if(realTorque>torque_list[torque40_Ncm]*0.90)
					{
						beep_time_out++;
						if(beep_time_out>10)//0.4S
						{
							beep_time_out=0;
							sendBuff= BUZZER_MODE_MOTOR_REVERSE;		
							xQueueSend(xQueueBeepMode, &sendBuff, 0);
						}	
					}			
					else 
					{		
						beep_time_out++;								
						if(beep_time_out>20)//0.5s
						{
							beep_time_out=0;
							sendBuff= BUZZER_MODE_MOTOR_REVERSE_LONG;		
							xQueueSend(xQueueBeepMode, &sendBuff, 0);
						}	
					}		
				}
			}	
		}				 
		else if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
		{			
			if(sys_param_un.device_param.apical_action_flag!=0&& GC_depth_vlaue(0, 1)<=(9+sys_param_un.device_param.ref_tine)&&sys_param_un.device_param.apexFunctionLoad!=0)				
			{
				;								
			}		 							
			else
			{	
				if(realTorque>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum])//MAX_TORQUE_UUPER_THRESHOLD*0.95)//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵婏箑鈻曢柛鈹惧亾濡炪倖甯掗崐褰掑吹閳ь剟鏌ｆ惔銏㈢叝闁告濞婃俊鎾磼閻愬瓨娅嗛梺�?咁潐閸旀洖鐨梻鍌欐祰椤宕曢崗鍏煎弿闁靛牆顦�?�褰掓煕椤愶絾绀堥柛娆愭崌閺屾盯濡烽敐鍛瀴闁诲繐绻掗�?鍝ユ閹烘鍋愮�?瑰�?�鍠氶崥瀣⒑閸濆�??顦柛鎾寸箞楠炲繘宕ㄩ娑樻�?闂佸憡娲﹂崑濠囨倵濞�?亝鈷掑〒�?�ｅ亾闁逞屽�?缁�??捇鍩為幒妤佺厱闁哄啠鍋撻柛銊ユ健閹即顢氶埀顒€鐣烽崼鏇ㄦ晢闁逞屽墰缁牓宕煎婵嗙秺閺佹劙宕熼鍛Τ婵犲痉銈�?毢闁稿繑锚椤繑绻濆�?�傦紲濠电偛�?欓崝妤呭Χ閺�?�簱鏀介柣鎰▕濡茶绻涢懠�?�€鏋庨柣锝夋敱鐎靛ジ�??堕幋鐘垫澑闂備�?�鎼ˇ�?�炴倶濠靛鍚归柟瀵�?�仧缁♀偓闂佹眹鍨藉�?�鐗庨梻浣藉亹閹�?挻鏅堕悾灞藉灊濠电姵鍑归�?宥�?�煟閹邦剦鍤熼柛娆忔濮婅�?�绱掑Ο鑽ゎ槬闂佺ǹ锕ゅ﹢閬嶅焵椤掍胶鍟查柟鍑ゆ�?4.0濠电姷鏁告慨鐑藉极閹间�?�纾婚柣鎰惈閸ㄥ倿鏌涢锝嗙缂佺姳鍗抽弻鐔兼⒒鐎靛壊妲紓浣哄Ь椤濡甸崟顖氱疀闁告挷鑳堕弳鐘绘⒒閸屾艾浜為柛銊ョ埣瀵濡搁埡鍌氫簽闂佺ǹ鏈粙鎴︻敂閿燂拷
				{		
					if(motor_settings.mode ==EndoModePositionToggle)
					{		
						overdCount++;	
						if(overdCount>35)	//10*60=600ms;0.6S,change for test
						{
							overdCount=0;	
							if(motor_settings.forward_position>-motor_settings.reverse_position)	
							{//
								motor_settings.mode=EndoModeKeepReverse;
							}
							else 
							{
								motor_settings.mode=EndoModeKeepForward;
							}
							motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
							motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
							App_MotorControl(MOTOR_MODE_RESTART);
						}		
					}				
				}	
				else if(realTorque<torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum]*0.6)//MAX_TORQUE_UUPER_THRESHOLD*0.6)//torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum])
				{						
					if(motor_settings.mode !=EndoModePositionToggle)
					{
						overdCount++;
						if(overdCount>40)//10*30=300ms;0.3S,change for test
						{												
							overdCount=0;							
							motor_settings.mode=EndoModePositionToggle;
							motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
							motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
							motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
							motor_settings.upper_threshold=torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum]*0.10f;//MAX_TORQUE_UUPER_THRESHOLD*0.10;//torque_list[torque40_Ncm]*0.10;
							motor_settings.lower_threshold=motor_settings.upper_threshold*0.6f;//torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum]*0.10;
							motor_settings.forward_position=motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition;
							motor_settings.reverse_position=motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition;
							App_MotorControl(MOTOR_MODE_RESTART);	
						}								
					}							 
				} //beep
				if(realTorque>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum]*0.8)//MAX_TORQUE_UUPER_THRESHOLD*0.80)//濠电姷鏁告慨鐢割敊閺嶎厼绐�?�俊銈呭暞閺嗘粍淇婇妶鍛�?�闁活厽鐟╅弻鐔兼倻濮楀棙鐣烽梺鍝勬噺缁诲牓�?婚弴锛勭杸閻�?綆浜�?崑鎾诲即閵忕姴鍤戞繛鎾村焹閸�?捇鏌″畝鈧崰鏍嵁閹达箑绠涢梻�?熺⊕椤斿�?绻濈喊妯活潑闁�?�鎳橀�?鍐閵堝懓鎽曢梺鍝�?储閸ㄥ綊鏌�??崶銊х瘈闂傚牊绋掔粊鈺備繆椤愩倕浠滄い顏勫暣婵″爼宕ㄩ�?�庡敹闂備胶�?悧鏇㈠Χ缁�??鍤曢悹鍥ㄧゴ濡插牊绻涢崱妯虹仼闁伙箑鐗撳Λ鍛�?敃閵忊€愁槱闂佸�?琚崝鎴︺€侀�?�?熸�?�闁跨噦鎷�
				{
					if(realTorque>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum])
					{
						if(beep_time_out>10)//0.2S
						{
							beep_time_out=0;
						}	
					}
					else if(realTorque>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum]*0.90)
					{
						beep_time_out++;
						if(beep_time_out>10)//0.4S
						{
							beep_time_out=0;
							sendBuff= BUZZER_MODE_MOTOR_REVERSE;		
							xQueueSend(xQueueBeepMode, &sendBuff, 0);
						}	
					}			
					else 
					{		
						beep_time_out++;							
						if(beep_time_out>20)//0.5s
						{
							beep_time_out=0;
							sendBuff= BUZZER_MODE_MOTOR_REVERSE_LONG;		
							xQueueSend(xQueueBeepMode, &sendBuff, 0);
						}	
					}	
				}
			}		  
		}			 	
		else 
		{
		 	 if(realTorque>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]*0.80)
			{	
				if(realTorque>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum])
				{
					if(beep_time_out>10)//0.2S
					{
						beep_time_out=0;
					}	
				}
				else if(realTorque>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]*0.90)
				{
					beep_time_out++;
					if(beep_time_out>10)//0.4S
					{
						beep_time_out=0;
						sendBuff= BUZZER_MODE_MOTOR_REVERSE;		
						xQueueSend(xQueueBeepMode, &sendBuff, 0);
					}	
				}
				else 
				{
					beep_time_out++;
					if(beep_time_out>20)//0.5s
					{
						beep_time_out=0;
						sendBuff= BUZZER_MODE_MOTOR_REVERSE_LONG;		
						xQueueSend(xQueueBeepMode, &sendBuff, 0);
					}	
				}					
			}
		}
	}			
}

/**
  * @brief 	WorkPageFlash
  * @param  
  * @retval none
  */
void WorkPageFlash(unsigned char workingMode, unsigned int systemTimeMs)//pageID (MENU_MOTOR_WORK_PAGE )only motor  (MENU_ONLY_APEX_PAGE) only apex (MENU_APEX_AND_MOTOR_PAGE) motor and apex  
{	
	unsigned short int realTorque;
	unsigned short int sendBuff;
	short int realApex;
	static unsigned int recTimeMs,timeout;
	realApex=GC_depth_vlaue(0, 1);//read	
	if(workingMode==MENU_MOTOR_WORK_PAGE)
	{		
		if(systemTimeMs-recTimeMs>10)
		{
			recTimeMs=systemTimeMs;			
			realTorque=GetRealTorque();	
			MotorRunModeAndTorque(realTorque, systemTimeMs);
			if(motor_status.status!=Status_STOP)	
			{	
				TorqueProgressBarFlash(systemTimeMs,1, realTorque);	
			}
		}
	}		
	else if(workingMode==MENU_ONLY_APEX_PAGE)
	{ 
		if(realApex==30)//no action
		{
			if(DeviceOffTimeManage(xTaskGetTickCount(),MINUTE_IDLE_TICKS*3))//8))//8  minute no fresh
			{
				sendBuff=run_button_long_press_signal;//
				xQueueSend(xQueueKeyMessage, &sendBuff, 0);		
			}
		}
		else
		{
			DeviceOffTimeManage(xTaskGetTickCount(),0);
		}
		GC_ControlMotor(realApex, systemTimeMs);
		timeout+=10;
		if(timeout>1000)//1s
		{
			timeout=0;
			#ifdef DEBUG_RTT
//			SEGGER_RTT_printf(0, "realApex=%d\r\n", realApex);	
			#endif
		}
		ApexProgressBarFlash(systemTimeMs,1, realApex);
	}			
	else if(workingMode==MENU_APEX_AND_MOTOR_PAGE)
	{
		if(10+recTimeMs<systemTimeMs)
		{				
			recTimeMs=systemTimeMs;
			realTorque=GetRealTorque();
			GC_ControlMotor(realApex, systemTimeMs);
			MotorRunModeAndTorque(realTorque, systemTimeMs);			
			FreshTorquePictureInApexAndMotorMode(0,0,realTorque,systemTimeMs);			
			if(realApex==30)//no action
			{
				timeout+=10;
				if(timeout>MINUTE_IDLE_TICKS)//*3) 	//3
				{
					timeout=0;
					sendBuff=run_button_press_signal;
					xQueueSend(xQueueKeyMessage, &sendBuff, 0);	
				}					
			}
			else
			{
				timeout=0;
			}	
			ApexProgressBarFlash(systemTimeMs,1, realApex);		
		}
	}	
} 
//=================CHARGING=====================================
/**
  * @brief 	Bat_ValueBlink
  * @param  systemTimeMs, startFlag 
  * @retval none
  */
static void Bat_ValueBlink(unsigned short int batValue,unsigned char blinkFlag)
{
	static unsigned short int history_batValue=0;	
	if(blinkFlag==0)
	{
		charg_bgm(40,2,1,batValue);
	}
	else 
	{	
		if(batValue<3000)
		{			
			 history_batValue=3000;						
		}
		else if(batValue<3450)
		{
			history_batValue=3000;		
		}
		else if(batValue<3650)
		{
			history_batValue=3400;			
		}
		else if(batValue<3950)
		{
			history_batValue=3600;	
		}
		else 
		{
//			if(batValue<4200) history_batValue=3600;
//			else history_batValue=batValue;	
			history_batValue=3900;			
		}		
		charg_bgm(40,2,1,history_batValue);
	}	
}
/**
  * @brief 	Display_CHARGING
  * @param  systemTimeMs, startFlag 
  * @retval none
  */
static void OLED_Display_CHARGING(unsigned short int batValue ,unsigned char startFlag)
{	
	static unsigned char blinkFlag;
	if(startFlag==0)
	{	
		charg_bgm(40,2,0,batValue);
	}
	else 
	{		
		if(get_charge_state()==RESET)		//闂傚倸鍊搁崐鎼佸磹�?��??海鐭嗗〒�?�ｅ亾妤犵偞鐗犻、鏇㈡晜閽樺�?缃曟繝鐢靛Т閿曘倗鈧凹鍣ｉ妴鍛村矗婢跺牅绨婚�?��?�㈡处閹哥偓鏅堕弴銏＄厸閻庯絺鏅濈粣鏃堟煛瀹€鈧崰鏍嵁閹达箑绠涢梻�?熺⊕椤斿�?绻濈喊妯活潑闁�?�鎳橀�?鍐閵堝懓鎽曢梺鍝�?储閸ㄥ綊鏌�??崶銊х瘈闂傚牊绋掔粊鈺備繆椤愩倕浠滄い顏勫暣婵″爼宕ㄩ�?�庡敹闂備胶�?悧婊堝储瑜旈敐鐐剁疀濞戞瑦鍎柣鐔哥懃鐎氼剛澹曢鐐粹拺闂傚牊鑳嗚ぐ鎺戠？闁哄�??鍎查崐闈浳旈敐鍛�?�闁抽攱鍨块弻娑樷攽閸℃浼岄梺绋块缁绘垿濡甸崟�?�ｆ晣闁绘ɑ褰�?�?瀣⒑缂佹ü绶遍柛鐘崇〒缁鈽�?�Ο閿嬵潔濠电偛妫欓崝鏍敂閻斿吋鈷掑ù锝堝Г绾爼鏌涢敐蹇曠暤妤犵偛绻橀�?鎾绘晸閿燂拷
		{
			blinkFlag =(blinkFlag==0)?1:0;	
			Bat_ValueBlink(batValue,blinkFlag);				
		}
		else
		{
			blinkFlag =0;
			Bat_ValueBlink(batValue,0);				
		}	
	}
}
/**
  * @brief  Page  turns
  * @param  pageID
  * @retval none
  */
static void MenuPageTurns(unsigned char pageID)
{	
	if(pageID>MENU_MAX_PAGE_NUM) return ;
	OLED_Clear_no_fresh();
	DeviceOffTimeManage(xTaskGetTickCount(),0);
	if(pageID==MENU_LOGO_PAGE)
	{
		OLED_Display_LOGO(0);//disp LOGO		
	}
	else if(pageID==MENU_HOME_PAGE)
	{		
		HomePageHandle(motor_setting_updata_signal ,ADJUST_MOTOR_PARAM_PROGRAM_NUM);
		disp_batValue(2,32,get_vbat_value());//prepare				
	}
	else if(pageID==MENU_SYSTEM_SET_PAGE)
	{
		#ifdef APEX_FUNCTION_EBABLE	
		SubmenuPageTurns(SETTING_FOR);
		#else
		SubmenuPageTurns(CW_ANGLE);
		#endif			
	}
	else if(pageID==MENU_MOTOR_WORK_PAGE)
	{		
		OLED_Display_WORK(MENU_MOTOR_WORK_PAGE,0);
	}
	else if(pageID==MENU_CHARGING_PAGE)
	{	
		OLED_Display_CHARGING(3300,0);		
	}
	else if(pageID==MENU_ONLY_APEX_PAGE)
	{	
		OLED_Display_WORK(MENU_ONLY_APEX_PAGE,0);	
		disp_batValue(2,32,get_vbat_value());//prepare		
	}
	else if(pageID==MENU_APEX_AND_MOTOR_PAGE)
	{	
		OLED_Display_WORK(MENU_APEX_AND_MOTOR_PAGE,0);		
	}
	else if(pageID==MENU_MAX_PAGE_NUM)//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎼€氥劑宕曢悽绋�?摕婵炴垶菤濡插牓鏌涘Δ鍐ㄤ�?闁�?�繐鐗嗛—鍐Χ閸℃顦俊鐐存綑閹芥粓骞戦�?�鐘闁靛繒�?�?懓鍨攽閳藉棗鐏犻柟纰卞亰閿濈偟浠︾粵瀣瘜闂侀潧鐗嗛幊鎰不娴煎瓨鐓熼柡宓�?�浠悗娈�?枦椤曆囧煡�?�舵劕顫呴柣�?活問閸熷鏌ｆ惔銈庢綈婵炲弶鐗曢埢鏃堟晝娓氬洦鈻岄梻浣告惈�?�跺�?宕滃┑瀣闁告侗鍨遍崰鍡涙煕閺囥劌浜為柨娑欐崌閺岋絾�?旈�?�鈺佹櫛闂侀潻缍嗛崳锝呯暦濠婂牊鍋勯梻鈧幇顔剧暰闂備�?�澹婇崑鍛�?弽�?�犲祦闁靛繆鈧尙绠氶梺闈涚墕閹锋垵�?忓Δ鍐＜闁�?�彃顑呴々�?�傜磼鏉堛劍宕岀�?规洘�?掗埢搴ㄥ箳濠靛�?�鏆梻鍌欒兌鏋い鎴濇�?�炴垿宕堕鈧拑鐔兼煏婵炵偓娅嗛柛瀣閺屾稓浠﹂崜�?�妲堝銈呴獜閹凤拷
	{
		OLED_Display_POWER_OFF(0);
	}
}
/*=======================Menu Manage Task===================================
			* @brief:  Menu Manage Task.
			* @param:  None
			* @retval: None
==============================================================================*/

void vAppMenuManageTask( void * pvParameters )
{ 
	task_notify_enum rec_Signal=null_signal;//
	ADJUST_select_NUM selectNum=ADJUST_MOTOR_PARAM_PROGRAM_NUM;
	SCREEN_SETTING_ENUM submenuPageNum=SETTING_FOR;
	static uint8_t menuPage=MENU_LOGO_PAGE,motor_run_cmd=0,motorOrApexSetFlag=0,exitApexFlag=1;
	confirm_state  apexMotorStartFlag =TRUE;//apex
	static unsigned 	short int  batDispValue;
	void *pMalloc=NULL;
	
	for(;;)
	{				
		if(xQueueReceive(xQueueKeyMessage, &rec_Signal, 0) == pdTRUE)
		{			
			MenuIdleTimeManage(xTaskGetTickCount(),0); //clear ticks
			DeviceOffTimeManage(xTaskGetTickCount(),0);//clear ticks	 			
			if(rec_Signal==low_power_signal)
			{			
			//beep
			//disp low power
			}
			else if(rec_Signal==run_button_long_press_signal)
			{	
				OLED_Clear_no_fresh();
				menuPage=MENU_MAX_PAGE_NUM;					
				OLED_Display_POWER_OFF(1);
				xSemaphoreGive(xSemaphoreDispRfresh);	
	 		}		
			else if(rec_Signal==power_off_signal)
			{
				OLED_Clear_no_fresh();
				menuPage=MENU_MAX_PAGE_NUM;
				OLED_Display_POWER_OFF(0);
				xSemaphoreGive(xSemaphoreDispRfresh);			
//				MenuDevicePowerOff(0);
			}	
		}						 
        switch(menuPage)
		{
		    case MENU_LOGO_PAGE:										 
				MenuPageTurns(menuPage);
				xSemaphoreGive(xSemaphoreDispRfresh);	
				#ifdef WDT_ENABLE
				//	1s
				xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);			 
				xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);						
				xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);
				xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);				
				#else
					vTaskDelay(1000);//	1s
				#endif				
				batDispValue=	get_vbat_value();	
				if(sys_param_un.device_param.use_p_num==10)
				{							
					menuPage=MENU_ONLY_APEX_PAGE;								
				}
				else
				{
					menuPage=MENU_HOME_PAGE;								
				}
				MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);//save data						
				MenuPageTurns(menuPage);
				xSemaphoreGive(xSemaphoreDispRfresh);
			break;
			case MENU_HOME_PAGE:								
				if(rec_Signal==run_button_press_signal)
				{		
					apexMotorStartFlag=FALSE;//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿劍�?�?い�?�㈢箰鐓ゆい蹇撳閻�?厼顪�?妶鍡樼闁瑰啿閰ｅ畷婊勩偅閸愨斁鎷洪梺鍛婄☉閿曘儱鐣峰畝鈧槐鎺楁偐瀹曞洤鈪瑰銈庡亜缁绘﹢骞栭崷�?�熷枂闁告洦鍋嗛敍蹇涙⒒娓氣偓濞佳勭仚闂佺ǹ瀛╅悡锟犲箖濡や降鍋呴柛鎰ㄦ杹閹�?櫣绱撴笟鍥х仭婵炲弶锚閳�?�秹宕ㄧ�?涙鍘甸梺鎯ф禋閸婂顢旈崶鈺�?缃曢梻鍌欑閹�?�繝宕濋敃鍌氱獥闁哄稁鍘介崑鍌涚箾閹存瑥鐏柣鎾存�?�閹﹢�?欓弶鎴濆Б闂佹悶鍊ら崜娆撴箒闂佺ǹ绻愰崥瀣暦鐏炵偓鍙忓┑鐘插鐢�?鏌熷畡鐗堝殗闁瑰�?鍋ゆ俊鐤槹闁逞屽�?閿曨亜顫忓ú�?�勪紶闁靛鍎涢敐澶�?厱闁哄啠鍋撻柣鐔村劦椤㈡岸鏁�?径濠傜獩婵犵數濮寸€氼噣�?侀崼婵冩斀妞ゆ梹鏋绘笟娑㈡煕閹惧娲存い銏＄懇瀹曞�?鈽�?�▎灞惧缂傚倸鍊烽悞锕傛�?闂佽绻�?畷�?�勫煘閹达富鏁婇柛婵嗗閸�??挸鈹戦崱鈺佹闂佸搫琚崕杈╃不閻熸噴褰掓晲閸涱喛纭�?闂佸憡蓱閹倸顫忛搹鍦煓闁�?ǹ瀛╅幏閬嶆⒑閹�?��?�鍝�?柡灞界Х椤т線鏌涢幘鏉戝摵闁诡啫鍐ｅ牚闁割偅绻勯崝锕�?�?�?妶鍡楃瑐闁煎啿鐖煎畷顖炲蓟閵夛�?�鍘甸梺鍛婂灟閸婃牜鈧�?鎷�
					MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);//save data							
					if(selectNum==ADJUST_MOTOR_PARAM_PROGRAM_NUM)
					{
						if(sys_param_un.device_param.auto_stop_flag==0&&sys_param_un.device_param.auto_start_flag==0&&sys_param_un.device_param.apical_action_flag==0)
						{	//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿鏌涘┑鍡楊伂妞ゎ偓绠撻弻娑㈠籍閳ь剟宕归崸妤冨祦闁告劦鍠栭～鍛存煏閸繃鍣�?紒鎰⊕缁绘繈鎮介�?�娴躲垽�?楀闂寸敖婵″弶鍔欏鎾閳锯偓閹风粯绻涙潏鍓ф偧闁稿簺鍊濆畷鐢稿焵椤掑�?鈷戦柟鑲╁仜閳ь剚鐗犻幃�?�鎮╅懡銈呯ウ闂婎偄娲︾粙鎺楀疾閹间焦鐓ラ柣鏇炲€�?�?氾拷
							menuPage = MENU_MOTOR_WORK_PAGE;
							if(motor_run_cmd==MOTOR_MODE_STOP) // motor control
							{
								motor_run_cmd=MOTOR_MODE_START;
							}									
							App_MotorControl(motor_run_cmd);	
						}
						else
						{
							if(sys_param_un.device_param.apexFunctionLoad!=0)//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿鏌涘┑鍡楊伂妞ゎ偓绠撻弻娑㈠籍閳ь剟宕归崸妤冨祦闁告劦鍠栭～鍛存煏閸繃鍣�?紒鎰⊕缁绘繈鎮介�?�娴躲垽�?楀闂寸敖婵″弶鍔欏鎾閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒冾潐濞叉﹢銆冮崨杈剧稏婵犲﹤鐗嗛悞�?亜閹哄棗浜惧銈嗘穿缂嶄線銆侀弴銏℃櫇闁逞屽墰缁粯绻濆�?�炰化闂佹悶鍎滈崘�?�堢崜婵＄偑鍊戦崕鑼崲閸繍�?�栨繛�?�簼椤ュ牊绻涢幋鐐跺妞わ絽鎼埞鎴﹀煡閸℃ぞ绨煎銈冨�?�閿曨亪濡存笟鈧�?�€宕煎┑瀣�?闂備礁鎼ú銊╁疮閸ф绠繛�?�簼閻撶喖鏌ｅΟ鍝勫�?闁煎壊浜弻娑㈠棘閼愁垰顏梺瀹狀嚙缁夊綊鐛崶�?�佸亱闁割偁鍨归�?宥�?�⒒娴ｅ憡鍟為柛�?戝灦瀹曟劙�??介鐔蜂壕婵鍋撶�?氾拷																						
							{		//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑呯粻娑樏归敐鍛�?�缂佸顥撶�?�鎾诲磼濞嗘埈妲銈嗗灥閹�?ǹ�?熼梺姹囧�?鏋柡鍕╁劦閺屽秷顧侀柛鎾跺枛瀵濡搁埡鍌氫簽闂佺ǹ鏈粙鎴︻敂閿燂拷							
								menuPage = MENU_APEX_AND_MOTOR_PAGE;
//								if(sys_param_un.device_param.auto_stop_flag==0)
//								{
									if(motor_run_cmd==MOTOR_MODE_STOP) // motor control
									{
										motor_run_cmd=MOTOR_MODE_START;
									}									
									App_MotorControl(motor_run_cmd);
//								}	
//								else 
//								{
//									motor_run_cmd=MOTOR_MODE_STOP;//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰琚︽俊鐐�?戦崹鐑�?�仚濡炪倧闄�?ぐ鍐箒濠电姴锕ょ€氼噣�?岄幒妤佺厵�?�ゆ棁顫楅崷顓犱笉婵炴垯鍨圭粻濠�?煛�?�跺鐏﹂柣锝囩帛娣囧﹪濡堕崶顬儵鏌涚�?ｎ偆銆掗柟骞�?灩椤粓鍩€椤掑�?宓侀柛鎰╁妷閸�?鏌涢銈�?瀻婵炲牏鍠栧娲濞戣鲸�?�嗛梻鍌�?鐎�?即銆侀�?�?熷亜闁�?�繐鐨烽幏缁樼�?�鏉堝墽鍒伴柟璇х節楠炲棝宕奸妷锔惧幈婵犵數�?�?崐濠�?春閿濆洠鍋撶憴鍕闁告梹鐟ラ锝嗙鐎ｅ灚鏅濋梺缁樻濞撹绔熼弴鐐╂斀妞ゆ梻鐡旈悞浠�??煕鐎ｎ偅灏甸柍�?�鍓氶悢顒勫�?閿燂�?
//								}											
							}	
							else 
							{															
								menuPage = MENU_MOTOR_WORK_PAGE;
								if(motor_run_cmd==MOTOR_MODE_STOP) // motor control
								{
									motor_run_cmd=MOTOR_MODE_START;
								}									
								App_MotorControl(motor_run_cmd);	
							}	
						}				
						MenuPageTurns(menuPage);	
					}
					else
					{//save motor param								
						selectNum=ADJUST_MOTOR_PARAM_PROGRAM_NUM;	
						HomePageHandle(motor_setting_updata_signal,ADJUST_MOTOR_PARAM_PROGRAM_NUM);									
					}										
				}				
				else if(rec_Signal==s_button_long_press_signal)
				{						
					menuPage = MENU_SYSTEM_SET_PAGE;//to set 
					submenuPageNum=SETTING_FOR;
					//motorOrApexSetFlag%=2;	
					motorOrApexSetFlag=0;	//濠电姷鏁告慨鐢割敊閺嶎厼绐�?�俊銈呭暞閺嗘粍淇婇妶鍛�?�闁活厽鐟╅弻鐔兼倻濮楀棙鐣烽梺鍝勬噺缁诲牓�?婚弴锛勭杸閻�?綆浜�?崑鎾诲即閵忕姴鍤戞繛鎾村焹閸�?捇鏌″畝鈧崰鏍嵁閹达箑绠涢梻�?熺⊕椤斿�?绻濈喊妯活潑闁�?�鎳橀�?鍐閵堝懓鎽曢梺鍝�?储閸ㄥ綊鏌�??崶銊х瘈闂傚牊绋掔粊鈺備繆椤愩倕浠滄い顏勫暣婵″爼宕ㄩ�?�庡敹闂備胶�?悧鏇㈠Χ缁�??鍤曢悹鍥ㄧゴ濡插牊绻涢崱妯虹仼闁伙箑鐗撳娲川婵犲啫顦╅梺鍛娒紞濠囧春閳ь剚銇勯幒鍡椾�?�濠电姭鍋撴い銊х樅R						
					MenuPageTurns(menuPage);
				}							
				else 
				{		
					if(rec_Signal==s_button_press_signal)
					{
						if(sys_param_un.device_param.use_p_num>5)
						{
							selectNum++;	
							selectNum%=(ADJUST_MOTOR_PARAM_DIRECTION+1);	
//							if(selectNum==ADJUST_MOTOR_PARAM_PROGRAM_NUM) selectNum=ADJUST_MOTOR_PARAM_SPEED;
							//if(selectNum==ADJUST_MOTOR_PARAM_TORQUE&&motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle) 
							//{
							//	selectNum=ADJUST_MOTOR_PARAM_DIRECTION;
							//}
							HomePageHandle(motor_setting_updata_signal,selectNum);									
						}	
					}
					else
					{							
						if(selectNum==ADJUST_MOTOR_PARAM_PROGRAM_NUM)
						{									
							if(rec_Signal==sub_button_press_signal)
							{
								#ifdef APEX_FUNCTION_EBABLE
									sys_param_un.device_param.use_p_num++;		
									sys_param_un.device_param.use_p_num%=11;//apex
								#else 
									sys_param_un.device_param.use_p_num++;		
									sys_param_un.device_param.use_p_num%=10;//no apex											
								#endif
								HomePageHandle(motor_setting_updata_signal,ADJUST_MOTOR_PARAM_PROGRAM_NUM);
							}
							else if(rec_Signal==add_button_press_signal)
							{										
								#ifdef APEX_FUNCTION_EBABLE
								if(sys_param_un.device_param.use_p_num==0)		
								{
									sys_param_un.device_param.use_p_num=10;//apex	
								}
								else
								{
									sys_param_un.device_param.use_p_num--;		
									sys_param_un.device_param.use_p_num%=11;//apex
									HomePageHandle(motor_setting_updata_signal,ADJUST_MOTOR_PARAM_PROGRAM_NUM);//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滄鐐寸墪鑿愭い鎺嗗亾闁逞屽�?閸ㄥ墎绮�?鍫涗汗闁圭儤鎸鹃崢顏呯節閻㈤潧孝缂佺�?�?￠幃鐐綇閵婏絼绨�?�梺鍝勬�?缁绘繈宕搹鍦＜缂備焦�?�?ú瀵糕偓瑙�?礀瀹曨剝鐏掗梺鍛婄箓鐎�?嘲顭�?幘缈犵箚闁绘劦浜滈埀顒佹礈閹广垽骞囬悧鍫濅函闂佺�?鎸哥�?�?帒顭�?弽�?�熺叄闊洦鍑瑰鎰偓瑙勬礀椤︾敻�??婚弴鐔虹闁割煈鍠栨慨鏇㈡⒑鐠囨煡鍙�?紒鐘崇墪椤繐煤椤忓�?绱堕梺鍛婃�?�濞佳囧礈閺夋娓�?�柕�?濋�?�炴﹢鏌涜箛鏃撹�?块柟顔�?�嵆椤㈡瑧鎹�?妸�?��?�戞繝鐢靛仦閸ㄥ爼鎮烽妷鈺佺劦�?�ゆ巻鍋撴い顓犲厴瀵濡舵径濠勭暢闂佸湱鍎ら崹鍨叏閺囩偐鏀介柣鎰级閸ｇ兘鏌涢妸銊︻�?��?�ゎ偄绻�?叅�?�ゅ繐瀚槐鍫曟⒑閸涘﹥澶�?柛瀣у亾闂佺ǹ顑嗛幑鍥х暦婵傜ǹ鍗抽柣鎰暜缁鳖噣姊绘担鍛婃儓缂佸绶�?畷鎴﹀焵椤掑�??鐓曢悗锝庡亝鐏忣厽銇�?锝囩疄妞ゃ垺顨婂畷鎺戔攦閻愵亜�?傛慨濠冩そ瀹曨偊宕熼鍛晧闂備�?�鎲″褰掑垂閸ф宓侀柛鎰靛枛椤懘鏌曢崼婵囧櫣缂佹劖绋掔换婵�?偨闂堟刀銏ゆ倵濮橀棿绨芥俊鍙�?��?�瀵挳濮�?閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒冾潐濞叉﹢銆冮崨杈剧稏婵犲﹤鐗嗛悞�?亜閹哄棗浜惧銈嗘穿缂嶄線銆侀弴銏℃櫇闁逞屽墰缁鎳￠妶鍥╋紳婵炶�?缍€閸�??倿骞�?┑鍐╃€�?梺闈涚箞閸婃牠鍩涢幋锔藉�?闁�?�厽�?掓俊鍏肩箾閸涱喖�?嶉柡宀�?鍠栧畷娆撳Χ閸℃浼�
								}	
								#else 
									sys_param_un.device_param.use_p_num--;		
									sys_param_un.device_param.use_p_num%=10;//no apex
								#endif							
							}											
							if(sys_param_un.device_param.use_p_num==10)			
							{
								menuPage = MENU_ONLY_APEX_PAGE;//to apex
								MenuPageTurns(menuPage);
							}																																
						}	
						else 
						{		
							if(sys_param_un.device_param.use_p_num<10)							
							{ 										
								HomePageHandle(rec_Signal,selectNum);
							}									
						}	
						if(rec_Signal==null_signal)//APEX闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿鏌涘┑鍡楊伂妞ゎ偓绠撻弻娑㈠籍閳ь剟宕归崸妤冨祦闁告劦鍠栭～鍛存煏閸繃鍣�?紒鎰⊕缁绘繈鎮介�?�娴躲垽�?楀闂寸敖婵″弶鍔欏鎾閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒侇問閸犳氨鍒掗�?�煎�?闁告洦鍨�?粻娑㈡煃鐟欏�?鍔ゅù婊堢畺閺屾稑鈽�?�Ο鍏兼喖闂佸�?鎳忕换鍫ュ蓟濞戞�?娌柛鎾椾�?�鍋撻幒妤佺厱婵炲�?�绻戦ˉ銏℃叏婵犲懏�?�犵紒杈ㄥ笒铻ｉ柤濮�?�?曞鎶芥⒒娴ｄ�?�鏀版い鏇熺矌閹广垹鈹戠�?ｎ亞鐣哄┑�?��?�仜閸�??挻銇�?姀鈥冲摵闁糕斁鍋撳銈嗗笒鐎氼剟鎷戦悢鍏肩厽闁哄倸鐏濋ˉ蹇斾繆閹绘帞绉烘鐐�?�墱閸掓帡宕楁径濠佸闂佸憡鍔�?�亸娆擃敇濞�?亝鈷掗柛灞剧懄缁佺�?�淇婂鐓庡闁诡喚鍋ら�?鍐磼濮橀硸鍞归梻浣�?�偠閸庢椽宕滃▎鎾村珔闁绘梻鍎ら崰鎰版煛閸屾侗鍎ラ柛銈嗘礋閺屾盯顢曢敐鍡欘槬闂佹悶鍔岄崐褰掑Φ閸曨垰绫嶉柛灞剧�?婢�?�洖�?�?妶鍡樷拹闁告梹鐟ラ～蹇曠磼濡顎撻梺鍛婄缚閸庤櫕顨欏┑锛勫亼閸婃垿宕曢弻銉ョ闁搞儜鍛濠德板€愰崑鎾淬亜椤愶絿�?掗柛鈹惧亾濡�?倖�?��?�崑鎾淬亜閺囶亞绉い銏℃�?�閺佸啴鍩€椤掑倻涓嶅┑鐘崇閸嬶綁鏌涢妷鎴濆暟妤犲洤顪�?妶鍐ㄥ�?�缂佽鐗嗛～蹇撁洪鍜佹濠电偞鍨堕懝楣冦�?傞崫鍕ㄦ斀闁宠棄妫楁�?�婵囥亜閵�?�儻�?柕鍡曠窔瀵噣宕奸悢鍛�?�仧闂備浇娉曢崳锕傚�?閿燂�?
						{	
							if(MenuIdleTimeManage(xTaskGetTickCount(),HOME_PARAM_IDLE_TICKS))
							{	
								#ifdef APEX_FUNCTION_EBABLE				
									submenuPageNum=SETTING_FOR;
								#else 
									submenuPageNum=CW_ANGLE;
								#endif								
								selectNum=ADJUST_MOTOR_PARAM_PROGRAM_NUM;								
								HomePageHandle(motor_setting_updata_signal,selectNum);
							}										
							if(sys_param_un.device_param.apexFunctionLoad!=0)
							{								
								if(sys_param_un.device_param.auto_start_flag!=0)
								{
									if(GC_depth_vlaue(0, 1)<27)
									{	
										if(apexMotorStartFlag==TRUE)	//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿鏌涘┑鍡楊伂妞ゎ偓绠撻弻娑㈠籍閳ь剟宕归崸妤冨祦闁告劦鍠栭～鍛存煏閸繃鍣�?紒鎰⊕缁绘繈鎮介�?�娴躲垽�?楀闂寸敖婵″弶鍔欏鎾閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒冾潐濞叉﹢銆冮崨杈剧稏婵犲﹤鐗嗛悞�?亜閹哄棗浜惧銈嗘穿缂嶄線銆侀弴銏℃櫇闁逞屽墰缁粯绻濆�?�炰化闂佹悶鍎滈崘�?�堢崜婵＄偑鍊戦崕鑼崲閸繍�?�栨繛�?�簼椤ュ牊绻涢幋鐐跺妞わ絽鎼埞鎴﹀煡閸℃ぞ绨煎銈冨�?�閿曨亪濡存笟鈧�?�€宕煎┑瀣�?闂備礁鎼ú銊╁疮閸ф绠繛�?�簼閻撶喖鏌ｅΟ鍝勫�?闁煎壊浜弻娑㈠棘鐠恒劎鍔悗娈�?枟閹倿鐛€ｎ喗鏅滈柣锝呰�?��?�炴劙�?�虹拠鎻掑毐缂傚秴妫欑粋宥�?�醇閺囩喎浜楅梺绋跨箳閳峰牆鈻撴禒瀣厽闁归偊鍨伴惃铏�?磼閻樺樊鐓奸柡宀�?鍠栭、娆撴偩瀹€鈧悡澶�?⒑閻熸�?�锛嶉柛瀣ㄥ�?栨穱濠囨倻閼恒儲娅嗙紓浣�?☉椤戝�?�骞�?悜鑺モ拻闁�?�本鑹鹃埀顒勵棑缁牊绗熼埀顒勭嵁閺嶎収鏁冮柨鏇楀亾缁�?儳缍婇弻娑㈩敃閿濆�?�顦ョ紒鐐劤缂嶅﹪�?婚垾鎰佸悑閹肩补鈧尙鐖遍梻浣呵归鍡涘箰閹间緤缍栨繝闈涱儛閺佸棝鏌涚仦鍓ф晼闁靛ň鏅滈埛鎴︽煕濠靛棗�?�い�?�畵閺屾盯�?埀顒勫垂閸ф宓侀柛鎰靛枛椤懘鏌曢崼婵囧櫣缂佹劖绋掔换婵�?偨闂堟刀銏ゆ倵濮橀棿绨芥俊鍙�?��?�瀵挳濮�?閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒冾潐濞叉﹢銆冮崨杈剧稏婵犲﹤鐗嗛悞�?亜閹哄棗浜惧銈嗘穿缂嶄線銆侀弴銏℃櫇闁逞屽墰缁粯绻濆�?�炰化闂佹悶鍎滈崘�?�堢崜婵＄偑鍊戦崕鑼崲閸繍�?�栨繛�?�簼椤ュ牊绻涢幋鐐跺妞わ絽鎼埞鎴﹀煡閸℃ぞ绨煎銈冨�?�閿曨亪濡存笟鈧�?�€宕掑☉妯硷紡闂佽崵鍋炵粙�?ュ�?�濠靛缍栫�?广儱顦伴埛鎴︽煕閿旇�??ㄦ俊鐐倐閺屾盯濡歌椤ｈ偐绱掗崒�?�毙ｉ柕�?秮瀹曟﹢鍩為悙顒€顏烘繝鐢靛仩閹活亞绱為埀顒€鈹戦鍝勨偓妤€鈽�?�悽绋块唶闁哄洨鍠撻崢閬嶆⒑缂佹◤�?�堝�?閹�?��?�鍙忕€广儱娲犻崑鎾斥枔閸喗鐏堥梺�?咁嚋缁辨洟骞戦�?�鐘斀闁糕檧鏅滈崓闈涱渻閵堝�?�绗掗柛瀣瀹曨剟鎮介崨濞炬嫽婵炶揪缍€婵倗娑甸崼鏇熺厱闁挎繂绻掗悾鍨�?殽閻�?尙绠婚柡浣�?�崌閺佹捇鏁撻敓锟�
										{	
//											apexMotorStartFlag=FALSE;//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿劍�?�?い�?�㈢箰鐓ゆい蹇撳閻�?厼顪�?妶鍡樼闁瑰啿閰ｅ畷婊勩偅閸愨斁鎷洪梺鍛婄☉閿曘儱鐣峰畝鈧槐鎺楁偐瀹曞洤鈪瑰銈庡亜缁绘﹢骞栭崷�?�熷枂闁告洦鍋嗛敍蹇涙⒒娓氣偓濞佳勭仚闂佺ǹ瀛╅悡锟犲箖濡や降鍋呴柛鎰ㄦ杹閹�?櫣绱撴笟鍥х仭婵炲弶锚閳�?�秹宕ㄧ�?涙鍘甸梺鎯ф禋閸婂顢旈崶鈺�?缃曢梻鍌欑閹�?�繝宕濋敃鍌氱獥闁哄稁鍘介崑鍌涚箾閹存瑥鐏柣鎾存�?�閹﹢�?欓弶鎴濆Б闂佹悶鍊ら崜娆撴箒闂佺ǹ绻愰崥瀣暦鐏炵偓鍙忓┑鐘插鐢�?鏌熷畡鐗堝殗闁瑰�?鍋ゆ俊鐤槹闁逞屽�?閿曨亜顫忓ú�?�勪紶闁靛鍎涢敐澶�?厱闁哄啠鍋撻柣鐔村劦椤㈡岸鏁�?径濠傜獩婵犵數濮寸€氼噣�?侀崼婵冩斀妞ゆ梹鏋绘笟娑㈡煕閹惧娲存い銏＄懇瀹曞�?鈽�?�▎灞惧缂傚倸鍊烽悞锕傛�?闂佽绻�?畷�?�勫煘閹达富鏁婇柛婵嗗閸�??挸鈹戦崱鈺佹闂佸搫琚崕杈╃不閻熸噴褰掓晲閸涱喛纭�?闂佸憡蓱閹倸顫忛搹鍦煓闁�?ǹ瀛╅幏閬嶆⒑閹�?��?�鍝�?柡灞界Х椤т線鏌涢幘鏉戝摵闁诡啫鍐ｅ牚闁割偅绻勯崝锕�?�?�?妶鍡楃瑐闁煎啿鐖煎畷顖炲蓟閵夛�?�鍘甸梺鍛婂灟閸婃牜鈧�?鎷�
											MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);//save data	
											if(GC_depth_vlaue(0, 1)>=0&&sys_param_un.device_param.apical_action_flag!=2)
											{
												if(motor_run_cmd==MOTOR_MODE_STOP) // motor control
												{
													motor_run_cmd=MOTOR_MODE_START;
												}	
												App_MotorControl(motor_run_cmd);			//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幍濡炪倖�?��?�崢褍危婵犳碍鐓�?悹鍥ㄧ叀椤庢鏌嶇拠鏌ュ弰妤犵偛顑呴埞鎴﹀幢濞嗗繐绗撴繝鐢靛Х閺佸憡鎱ㄩ悽鍓叉晩闁哄稁鍘肩粣妤呮煙閻戞﹩娈旈柣銈夌畺閺岋絽螣濞嗘儳娈梺钘�?�暟閸犳牠�??婚弴鐔虹闁绘劦鍓氶悵鏇㈡⒑缁�??鍎忛悗姘嵆瀵鈽夊Ο閿嬵潔濠�?�喗顨呴悧鍡樻叏濞戞瑧绠鹃悗鐢登�?�?�?曟倵濮橆厽绶叉い�?�㈢箰鐓ゆい蹇撴噹娴狀參�?�洪�?鍕垫Ч閻庣瑳鍥舵晩闁逞屽墴濮婄粯鎷呴悜妯烘畬闂佹悶鍊�?悧鐘荤嵁�?囨稒鏅搁柨鐕傛�??
											}														
											menuPage = MENU_APEX_AND_MOTOR_PAGE;//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿鏌涘┑鍡楊伂妞ゎ偓绠撻弻娑㈠籍閳ь剟宕归崸妤冨祦闁告劦鍠栭～鍛存煏閸繃鍣�?紒鎰⊕缁绘繈鎮介�?�娴躲垽�?楀闂寸敖婵″弶鍔欏鎾閳锯偓閹风粯绻涙潏鍓ф偧闁稿簺鍊濆畷鐢稿焵椤掑�?鈷戦柟鑲╁仜閳ь剚鐗犻幃�?�鎮╅懡銈呯ウ闂婎偄娲︾粙鎺楀疾閹间焦鐓ラ柣鏇炲€�?�?氾拷
											MenuPageTurns(menuPage);
										}
									}	
                					else 
									{
										apexMotorStartFlag=TRUE;//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿鏌涘┑鍡楊伂妞ゎ偓绠撻弻娑㈠籍閳ь剟宕归崸妤冨祦闁告劦鍠栭～鍛存煏閸繃鍣�?紒鎰⊕缁绘繈鎮介�?�娴躲垽�?楀闂寸敖婵″弶鍔欏鎾閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒冾潐濞叉﹢銆冮崨杈剧稏婵犲﹤鐗嗛悞�?亜閹哄棗浜惧銈嗘穿缂嶄線銆侀弴銏℃櫇闁逞屽墰缁粯绻濆�?�炰化闂佹悶鍎滈崘�?�堢崜婵＄偑鍊戦崕鑼崲閸繍�?�栨繛�?�簼椤ュ牊绻涢幋鐐跺妞わ絽鎼埞鎴﹀煡閸℃ぞ绨煎銈冨�?�閿曨亪濡存笟鈧�?�€宕煎┑瀣�?闂備礁鎼ú銊╁疮閸ф绠繛�?�簼閻撶喖鏌ｅΟ鍝勫�?闁煎壊浜弻娑㈠棘鐠恒劎鍔悗娈�?枟閹倿鐛€ｎ喗鏅滈柣锝呰�?��?�炴劙�?�虹拠鎻掑毐缂傚秴妫欑粋宥�?�醇閺囩喎浜楅梺绋跨箳閳峰牆鈻撴禒瀣厽闁归偊鍨伴惃铏�?磼閻樺樊鐓奸柡灞稿墲閹峰懐鎲�?崟�?�わ紦闂備浇妗ㄩ悞锕傚�?�閸�?剙鏋侀柟鍓х帛閺�?悂鏌ㄩ悤鍌涘�?
									}												
								}						
							}									
							else
							{
								apexMotorStartFlag=TRUE;//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿鏌涘┑鍡楊伂妞ゎ偓绠撻弻娑㈠籍閳ь剟宕归崸妤冨祦闁告劦鍠栭～鍛存煏閸繃鍣�?紒鎰⊕缁绘繈鎮介�?�娴躲垽�?楀闂寸敖婵″弶鍔欏鎾閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒冾潐濞叉﹢銆冮崨杈剧稏婵犲﹤鐗嗛悞�?亜閹哄棗浜惧銈嗘穿缂嶄線銆侀弴銏℃櫇闁逞屽墰缁粯绻濆�?�炰化闂佹悶鍎滈崘�?�堢崜婵＄偑鍊戦崕鑼崲閸繍�?�栨繛�?�簼椤ュ牊绻涢幋鐐跺妞わ絽鎼埞鎴﹀煡閸℃ぞ绨煎銈冨�?�閿曨亪濡存笟鈧�?�€宕煎┑瀣�?闂備礁鎼ú銊╁疮閸ф绠繛�?�簼閻撶喖鏌ｅΟ鍝勫�?闁煎壊浜弻娑㈠棘鐠恒劎鍔悗娈�?枟閹倿鐛€ｎ喗鏅滈柣锝呰�?��?�炴劙�?�虹拠鎻掑毐缂傚秴妫欑粋宥�?�醇閺囩喎浜楅梺绋跨箳閳峰牆鈻撴禒瀣厽闁归偊鍨伴惃铏�?磼閻樺樊鐓奸柡灞稿墲閹峰懐鎲�?崟�?�わ紦闂備浇妗ㄩ悞锕傚�?�閸�?剙鏋侀柟鍓х帛閺�?悂鏌ㄩ悤鍌涘�?
							}
						}																	
					}								
					if(DeviceOffTimeManage(xTaskGetTickCount(),DEVICE_POWER_OFF_TICKS))//3 minute
					{
						OLED_Clear_no_fresh();					
						OLED_Display_POWER_OFF(1);
						xSemaphoreGive(xSemaphoreDispRfresh);	
						#ifdef WDT_ENABLE//	1s								
						xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
						vTaskDelay(MAX_WDT_FEED_TIME_MS);
						xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
						vTaskDelay(MAX_WDT_FEED_TIME_MS);						
						#else
						vTaskDelay(600);//1s
						#endif
						MenuDevicePowerOff(0);
					}		
				}	
			break;
			case MENU_SYSTEM_SET_PAGE:					
				if(rec_Signal==s_button_long_press_signal||rec_Signal==run_button_press_signal)
				{													
					SubmenuPageHandle(submenuPageNum,motor_setting_updata_signal,0,0);
					menuPage = MENU_HOME_PAGE;//exit						
					MenuPageTurns(menuPage);
				}
				else 
				{												
					if(rec_Signal==s_button_press_signal)
					{
						#ifdef APEX_FUNCTION_EBABLE	
						if(motorOrApexSetFlag==0)//motor
						{
							if(submenuPageNum==SETTING_FOR)
							{								
								submenuPageNum=CW_ANGLE;//
							}
							else
							{
								submenuPageNum++;
								submenuPageNum%=MAX_SUBMENU;
							}								
						}
						else//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑嗛崐閿嬨亜閹哄棗浜惧銈呴獜閹凤拷
						{
							if(submenuPageNum==SETTING_FOR)
							{								
								submenuPageNum=AUTO_START;//
							}							
							else
							{ 
								submenuPageNum++;
								submenuPageNum%=(APICAL_OFFESET+1);
							}								
						}	
						#else
						submenuPageNum++;
						submenuPageNum%=MAX_SUBMENU;
						if(submenuPageNum<CW_ANGLE) submenuPageNum=CW_ANGLE;
						#endif															
						SubmenuPageTurns(submenuPageNum);
					}		
					else
					{		
            			if(rec_Signal==add_button_press_signal||rec_Signal==sub_button_press_signal)	
						{
							if(submenuPageNum==SETTING_FOR)
							{
								motorOrApexSetFlag++;
								motorOrApexSetFlag%=2;
							}									
						}								
						SubmenuPageHandle(submenuPageNum,rec_Signal,xTaskGetTickCount(),1+motorOrApexSetFlag);	
						if(rec_Signal==null_signal)
						{
							if(MenuIdleTimeManage(xTaskGetTickCount(),MENU_IDLE_TICKS))
							{
								#ifdef APEX_FUNCTION_EBABLE	
								submenuPageNum=SETTING_FOR;
								#else 
								submenuPageNum=CW_ANGLE;
								#endif			
								SubmenuPageHandle(submenuPageNum,motor_setting_updata_signal,0,0);
								menuPage = MENU_HOME_PAGE;//exit						
								MenuPageTurns(menuPage);
							}
						}											
					}	
				}						
			break;
			case MENU_MOTOR_WORK_PAGE:
				if(rec_Signal==run_button_press_signal||rec_Signal==low_power_signal)
				{							
					if(motor_run_cmd==MOTOR_MODE_START) //motor control
					{						
						motor_run_cmd=MOTOR_MODE_STOP;
					}	
					App_MotorControl(motor_run_cmd);						
					menuPage=MENU_HOME_PAGE;		
					MenuPageTurns(menuPage);
				}							
				WorkPageFlash(MENU_MOTOR_WORK_PAGE,xTaskGetTickCount());	
				if(sys_param_un.device_param.apexFunctionLoad!=0)//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿鏌涘┑鍡楊伂妞ゎ偓绠撻弻娑㈠籍閳ь剟宕归崸妤冨祦闁告劦鍠栭～鍛存煏閸繃鍣�?紒鎰⊕缁绘繈鎮介�?�娴躲垽�?楀闂寸敖婵″弶鍔欏鎾閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒冾潐濞叉﹢銆冮崨杈剧稏婵犲﹤鐗嗛悞�?亜閹哄棗浜惧銈嗘穿缂嶄線銆侀弴銏℃櫇闁逞屽墰缁粯绻濆�?�炰化闂佹悶鍎滈崘�?�堢崜婵＄偑鍊戦崕鑼崲閸繍�?�栨繛�?�簼椤ュ牊绻涢幋鐐跺妞わ絽鎼埞鎴﹀煡閸℃ぞ绨煎銈冨�?�閿曨亪濡存笟鈧�?�€宕煎┑瀣�?闂備礁鎼ú銊╁疮閸ф绠繛�?�簼閻撶喖鏌ｅΟ鍝勫�?闁煎壊浜弻娑㈠棘閼愁垰顏梺瀹狀嚙缁夊綊鐛崶�?�佸亱闁割偁鍨归�?宥�?�⒒娴ｅ憡鍟為柛�?戝灦瀹曟劙�??介鐔蜂壕婵鍋撶�?氾拷
				{					
					menuPage=MENU_APEX_AND_MOTOR_PAGE;	//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滄鐐寸墪鑿愭い鎺嗗亾闁逞屽�?閸ㄥ墎绮�?鍫涗汗闁圭儤鎸鹃崢顏呯節閻㈤潧孝缂佺�?�?￠幃鐐綇閵婏絼绨�?�梺鍝勬�?缁绘繈宕搹鍦＜缂備焦�?�?ú瀵糕偓瑙�?礀瀹曨剝鐏掗梺鍛婄箓鐎�?嘲顭�?幘缈犵箚闁绘劦浜滈埀顒佹礈閹广垽骞囬悧鍫濅函闂佺�?鎸哥�?�?帒顭�?弽�?�熺叄闊洦鍑瑰鎰偓瑙勬礀椤︾敻�??婚弴鐔虹闁割煈鍠栨慨鏇㈡⒑鐠囨煡鍙�?紒鐘崇墪椤繐煤椤忓�?绱堕梺鍛婃�?�濞佳囧礈閺夋娓�?�柕�?濋�?�炴﹢鏌涜箛鏃撹�?块柟顔�?�嵆椤㈡瑧鎹�?妸�?��?�戞繝鐢靛仦閸ㄥ爼鎮烽妷鈺佺劦�?�ゆ巻鍋撴い顓犲厴瀵濡舵径濠勭暢闂佸湱鍎ら崹鍨叏閺囩偐鏀介柣鎰级閸ｇ兘鏌涢妸銊︻�?��?�ゎ偄绻�?叅�?�ゅ繐瀚槐鍫曟⒑閸涘﹥澶�?柛瀣у亾闂佺ǹ顑嗛幑鍥х暦婵傜ǹ鍗抽柣鎰暜缁鳖噣姊绘担鍛婃儓缂佸绶�?畷鎴﹀焵椤掑�??鐓曢悗锝庡亝鐏忣厽銇�?锝囩疄妞ゃ垺顨婂畷鎺戔攦閻愵亜�?傛慨濠冩そ瀹曨偊宕熼鍛晧闂備�?�鎲″褰掑垂閸ф宓侀柛鎰靛枛椤懘鏌曢崼婵囧櫣缂佹劖绋掔换婵�?偨闂堟刀銏ゆ倵濮橀棿绨芥俊鍙�?��?�瀵挳濮�?閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒冾潐濞叉﹢銆冮崨杈剧稏婵犲﹤鐗嗛悞�?亜閹哄棗浜惧銈嗘穿缂嶄線銆侀弴銏℃櫇闁逞屽墰缁粯绻濆�?�炰化闂佹悶鍎滈崘�?�堢崜婵＄偑鍊戦崕鑼崲閸繍�?�栨繛�?�簼椤ュ牊绻涢幋鐐跺妞わ絽鎼埞鎴﹀煡閸℃ぞ绨煎銈冨�?�閿曨亪濡存笟鈧�?�€宕煎┑瀣�?闂備礁鎼ú銊╁疮閸ф绠繛�?�簼閻撶喖鏌ｅΟ鍝勫�?闁煎壊浜弻娑㈠棘鐠恒劎鍔悗娈�?枟閹倿鐛€ｎ喗鏅滈柣锝呰�?��?�炴劙�?�虹拠鎻掑毐缂傚秴妫欑粋宥�?�醇閺囩喎浜楅梺绋跨箳閳峰牆鈻撴禒瀣厽闁归偊鍨伴惃铏�?磼閻樺樊鐓奸柡宀�?鍠栭、娆撴偩瀹€鈧悡澶�?⒑閻熸�?�锛嶉柛瀣ㄥ�?栨穱濠囨倻閼恒儲娅嗙紓浣�?☉椤戝�?�骞�?悜鑺モ拻闁�?�本鑹鹃埀顒勵棑缁牊绗熼埀顒勭嵁閺嶎収鏁冮柨鏇楀亾缁�?儳缍婇弻娑㈩敃閿濆�?�顦ョ紒鐐劤缂嶅﹪�?婚垾鎰佸悑閹肩补鈧尙鐖遍梻浣呵归鍡涘箰閹间緤缍栨繝闈涱儛閺佸棝鏌涚仦鍓ф晼闁靛ň鏅滈埛鎴︽煕濠靛棗�?�い�?�畵閺屾盯�?埀顒勫垂閸ф宓侀柛鎰靛枛鍥存繝銏ｆ硾閿曪箓鏁嶅⿰鍐ｆ斀闁绘劖�?�欓悘锕傛煥閺囨娅嗛悗闈涖偢閺佹捇鏁撻敓锟�?	
					MenuPageTurns(menuPage);
				}													
				 break;
			case MENU_ONLY_APEX_PAGE:	
				if(rec_Signal==sub_button_press_signal)
				{	
					sys_param_un.device_param.use_p_num++;
					sys_param_un.device_param.use_p_num%=11;
					menuPage=MENU_HOME_PAGE;		
					MenuPageTurns(menuPage);
				}
				else if(rec_Signal==add_button_press_signal)
				{												
					sys_param_un.device_param.use_p_num--;		
					sys_param_un.device_param.use_p_num%=11;//apex	
					menuPage=MENU_HOME_PAGE;		
					MenuPageTurns(menuPage);
				}							
				else if(rec_Signal==s_button_long_press_signal)
				{								
					menuPage = MENU_SYSTEM_SET_PAGE;//to set
					submenuPageNum=SETTING_FOR;					
					MenuPageTurns(menuPage);							
				}												
				else
				{	 						
					WorkPageFlash(MENU_ONLY_APEX_PAGE,xTaskGetTickCount());	
					if(sys_param_un.device_param.apexFunctionLoad!=0&&exitApexFlag!=0)
					{
						exitApexFlag=0;//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿�?归崶锝傚亾閾忣偆浜愰梻浣告惈閹虫劖绻涢埀顒侇殽閻�?�?�ユ�?�冨嵆瀹曘劑顢橀悢琛″亾閸愭祴鏀介柍钘�?�閻忋儲淇婂鐓庡缂佹梻鍠栧鎾閳锯偓閹锋椽鏌ｉ悩鍏呰埅闁告柨鑻埢宥�?�箛閻�?�牏鍘甸梺鍛婂灟閸婃牜鈧�?鎷�
					}
					if(sys_param_un.device_param.apexFunctionLoad==0&&exitApexFlag==0)//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?節瀹曞爼鍩℃担鍦簴缂傚倷鑳舵繛鈧紒鐘崇墵瀵鈽夐�?�鐘栥劑鏌曡箛濠傚⒉闁绘繃鐗�?�穱濠囨倷妫版繃缍堟繝鐢靛仜閿曨�?鐛崘銊ф殝闁逛絻娅曢悗濠�?椤愩垺澶勯柟灏栨櫆缁傛帡鎮℃惔顔藉瘜闂侀潧鐗嗛幊鎰不娴煎瓨鐓熼柡�?�庡亜鐢埖銇勯銏㈢闁圭厧婀遍幉鎾礋椤愩倧绱￠梻鍌欑窔濞佳勭仚闂佺ǹ瀛╅悡锟犲箖濡や降鍋呴柛鎰ㄦ杹閹锋椽姊洪崨濠�?畵閻庢氨鍏樺畷鏇＄疀閺傝绨诲銈嗘尵婵挳宕㈤幘�?�界厽婵炴垵宕弸锔剧磼閻樺�?鈽�?�柍钘�?�槸閳�?�氦绠涙繝鍌欏婵犵數濮电喊宥�?�偂閻�?�粯鐓欐い鎾跺枎缁�?�帡鏌涢�?鈧划�?�囨崲濞戙垹宸濇い鎰╁灮娴煎牆鈹戦�?锋敾婵＄偠妫�?悾鐑筋敃閿曗偓缁�?瀣亜閹邦喖鏋庡ù婊堢畺閺屾盯顢曢敐鍡欘槬缂佺偓鍎崇紞濠囧蓟閳ユ剚鍚�??幖绮光偓鑼埍闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿鏌涘┑鍡楊伂妞ゎ偓绠撻弻娑㈠籍閳ь剟宕归崸妤冨祦闁告劦鍠栭～鍛存煏閸繃鍣�?紒鎰⊕缁绘繈鎮介�?�娴躲垽�?楀闂寸敖婵″弶鍔欏鎾閳锯偓閹风粯绻涙潏鍓ф偧闁稿簺鍊濆畷鐢稿焵椤掑�?鈷戦柟鑲╁仜閳ь剚鐗犻幃�?�鎮╅懡銈呯ウ闂婎偄娲︾粙鎺楀疾閹间焦鐓ラ柣鏇炲€�?�?氾拷,闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滄鐐寸墪鑿愭い鎺嗗亾闁逞屽�?閸ㄥ墎绮�?鍫涗汗闁圭儤鎸鹃崢顏呯節閻㈤潧孝缂佺�?�?￠幃鐐綇閵婏絼绨�?�梺鍝勬�?缁绘繈宕搹鍦＜缂備焦�?�?ú瀵糕偓瑙�?礀瀹曨剝鐏掗梺鍛婄箓鐎�?嘲顭�?幘缈犵箚闁绘劦浜滈埀顒佹礈閹广垽骞囬悧鍫濅函闂佺�?鎸哥�?�?帒顭�?弽�?�熺叄闊洦鍑瑰鎰偓瑙勬礀椤︾敻�??婚弴鐔虹闁割煈鍠栨慨鏇㈡⒑鐠囨煡鍙�?紒鐘崇墪椤繐煤椤忓�?绱堕梺鍛婃�?�濞佳囧礈閺夋娓�?�柕�?濋�?�炴﹢鏌涜箛鏃撹�?块柟顔�?�嵆椤㈡瑧鎹�?妸�?��?�戞繝鐢靛仦閸ㄥ爼鎮烽妷鈺佺劦�?�ゆ巻鍋撴い顓犲厴瀵濡舵径濠勭暢闂佸湱鍎ら崹鍨叏閺囩偐鏀介柣鎰级閸ｇ兘鏌涢妸銊︻�?��?�ゎ偄绻�?叅�?�ゅ繐瀚槐鍫曟⒑閸涘﹥澶�?柛瀣у亾闂佺ǹ顑嗛幑鍥х暦婵傜ǹ鍗抽柣鎰暜缁鳖噣姊绘担鍛婃儓缂佸绶�?畷鎴﹀焵椤掑�??鐓曢悗锝庡亝鐏忣厽銇�?锝囩疄妞ゃ垺顨婂畷鎺戔攦閻愵亜�?傛慨濠冩そ瀹曨偊宕熼鍛晧闂備�?�鎲″褰掑垂閸ф宓侀柛鎰靛枛椤懘鏌曢崼婵囧櫣缂佹劖绋掔换婵�?偨闂堟刀銏ゆ倵濮橀棿绨芥俊鍙�?��?�瀵挳濮�?閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒冾潐濞叉﹢銆冮崨杈剧稏婵犲﹤鐗嗛悞�?亜閹哄棗浜惧銈嗘穿缂嶄線銆侀弴銏℃櫇闁逞屽墰缁粯绻濆�?�炰化闂佹悶鍎滈崘�?�堢崜婵＄偑鍊戦崕鑼崲閸繍�?�栨繛�?�簼椤ュ牊绻涢幋鐐跺妞わ絽鎼埞鎴﹀煡閸℃ぞ绨煎銈冨�?�閿曨亪濡存笟鈧�?�€宕煎┑瀣�?闂備礁鎼ú銊╁疮閸ф绠繛�?�簼閻撶喖鏌ｅΟ鍝勫�?闁煎壊浜弻娑㈠棘鐠恒劎鍔悗娈�?枟閹倿鐛€ｎ喗鏅滈柣锝呰�?��?�炴劙�?�虹拠鎻掑毐缂傚秴妫欑粋宥�?�醇閺囩喎浜楅梺绋跨箳閳峰牆鈻撴禒瀣厽闁归偊鍨伴惃铏�?磼閻樺崬宓嗛柡宀�?�?閺佹劘绠涢弴鐘垫婵°倗濮烽崑娑氭崲濡�?�鍤曟い鏇�?�亾闁糕斁鍋撳銈嗗笒鐎氼參宕曟惔鈧簻闁哄稁鍋勬禍濂告煃瑜滈崗娑�?濮橆剦鍤曞ù鐘差儏鎯熼梺鎸庢煥�?�у€熷€存繝纰�?�磿閸�??垿宕愰弽顓炵�?鐟滃繐�?�ユ繝鍥ㄥ癄濠㈣泛妫欓～宥�?�偡濠婂�?鐓ユい�?�㈢箲閵�?妇鎲楅妶鍕潖闂備�?�婀遍崕銈囨崲閸愵啟澶愬冀椤撶啿鎷虹紓浣割儓濞夋洜�?婚弻銉︾叆闁哄洦锚閻忊晜銇勯弴�?�嗙М妞ゃ垺娲熼弫鍐焵椤掑倻涓嶅┑鐘崇閸嬶綁鏌涢妷�?�荤盎闁汇劎鍎ら妵鍕�?�閼愁垰顏銈庝簻閸熷瓨淇婇崼鏇炲耿婵☆垱妞介悗铏�?磽閸屾瑦绁版い鏇嗗應鍋撳鐓庡�?�缂侇喗鐟﹀鍕箛椤掑�?鎲伴梻渚�?娼ч¨鈧┑鈥�?喘瀹曘垽宕归锛勭畾闂佺粯鍔曞Ο濠囧磿韫囨拹鏃堟偐閾忣偄鈧劗鈧娲忛崹浠�?嵁濮椻偓椤㈡瑩鎮剧仦钘�?�婵犵數鍋犻幓顏嗙礊閳ь剚銇�?銏╂Ц闁伙絽鍢查悾婵�?礋椤掑倸骞�?┑鐐舵彧缂嶁偓�?�ゎ偄顦悾宄扮暆閸曨剛鍘告繛杈剧秬椤�?鐣峰畝鍕厸濞达綀�?夊畷宀�?煛娴ｈ宕岄柡浣规崌閺佹捇鏁撻敓锟�
					{
						exitApexFlag=1;
						//sys_param_un.device_param.use_p_num=0;
						//menuPage=MENU_HOME_PAGE;		
						//MenuPageTurns(menuPage);
					}	
				}													
			break;
			case MENU_APEX_AND_MOTOR_PAGE:
				if(rec_Signal==run_button_press_signal||rec_Signal==low_power_signal)
				{	
					apexMotorStartFlag=FALSE;//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿劍�?�?い�?�㈢箰鐓ゆい蹇撳閻�?厼顪�?妶鍡樼闁瑰啿閰ｅ畷婊勩偅閸愨斁鎷洪梺鍛婄☉閿曘儱鐣峰畝鈧槐鎺楁偐瀹曞洤鈪瑰銈庡亜缁绘﹢骞栭崷�?�熷枂闁告洦鍋嗛敍蹇涙⒒娓氣偓濞佳勭仚闂佺ǹ瀛╅悡锟犲箖濡や降鍋呴柛鎰ㄦ杹閹�?櫣绱撴笟鍥х仭婵炲弶锚閳�?�秹宕ㄧ�?涙鍘甸梺鎯ф禋閸婂顢旈崶鈺�?缃曢梻鍌欑閹�?�繝宕濋敃鍌氱獥闁哄稁鍘介崑鍌涚箾閹存瑥鐏柣鎾存�?�閹﹢�?欓弶鎴濆Б闂佹悶鍊ら崜娆撴箒闂佺ǹ绻愰崥瀣暦鐏炵偓鍙忓┑鐘插鐢�?鏌熷畡鐗堝殗闁瑰�?鍋ゆ俊鐤槹闁逞屽�?閿曨亜顫忓ú�?�勪紶闁靛鍎涢敐澶�?厱闁哄啠鍋撻柣鐔村劦椤㈡岸鏁�?径濠傜獩婵犵數濮寸€氼噣�?侀崼婵冩斀妞ゆ梹鏋绘笟娑㈡煕閹惧娲存い銏＄懇瀹曞�?鈽�?�▎灞惧缂傚倸鍊烽悞锕傛�?闂佽绻�?畷�?�勫煘閹达富鏁婇柛婵嗗閸�??挸鈹戦崱鈺佹闂佸搫琚崕杈╃不閻熸噴褰掓晲閸涱喛纭�?闂佸憡蓱閹倸顫忛搹鍦煓闁�?ǹ瀛╅幏閬嶆⒑閹�?��?�鍝�?柡灞界Х椤т線鏌涢幘鏉戝摵闁诡啫鍐ｅ牚闁割偅绻勯崝锕�?�?�?妶鍡楃瑐闁煎啿鐖煎畷顖炲蓟閵夛�?�鍘甸梺鍛婂灟閸婃牜鈧�?鎷�
					if(motor_run_cmd == MOTOR_MODE_START) //motor control
					{						
						motor_run_cmd=MOTOR_MODE_STOP;
					}		
					menuPage=MENU_HOME_PAGE;//page change					
					MenuPageTurns(menuPage);
					App_MotorControl(MOTOR_MODE_STOP);						
				 	//if(rec_Signal==run_button_press_signal&&GC_depth_vlaue(0, 1)<=(3+sys_param_un.device_param.ref_tine)&&sys_param_un.device_param.apical_action_flag==2)//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿�?归崶锝傚亾閾忣偆浜愰梻浣告惈閹虫劖绻涢埀顒侇殽閻�?�?�ユ�?�冨嵆瀹曘劑顢橀悢琛″亾閸愭祴鏀介柍钘�?�閻忋儲淇婂鐓庡缂佹梻鍠栧鎾閳锯偓閹锋椽鏌ｉ悩鍏呰埅闁告柨鑻埢宥�?�箛閻�?�牏鍘甸梺鍛婂灟閸婃牜鈧�?鎷�
				//	{//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閸婄敻�?峰▎蹇擃仾缂佲偓閸愵喗鐓曢柕濞у�??姣堥梺闈涚墱娴滄繄鈧潧銈�?��?曞�?�閻欌偓濡插綊�?��?�崒姘偓鎼佹偋婵犲�?�?欓柟娆¤顦埞鎴�?偓锝庡亐閹峰�?�虹粙鎸庢拱闁荤噦濡囩划濠囨偋閸垻顔曟繝銏ｆ硾椤戝棛绮堢€ｎ兘鍋撶憴鍕；闁告濞婇悰顕€宕堕鈧痪褔鏌涢…鎴濇灈婵炲牄鍊曢埞鎴︽偐閸偅姣勬繝娈�?枤閺佸骞婂┑瀣�?鐟滃繑绋夊鍡愪簻闁哄稁鍋勬禒锕傛煟閹捐泛鏋涢柡宀�?鍠愬蹇涘�?�瑜嶉崺宀�?偠濮橆厼鍝烘慨濠冩そ瀹曨偊宕熼鈧▍銈夋⒑鐠団�?�?灈闁稿﹤鐏濋锝嗙節�?橆厽娅㈤梺缁樓圭亸娆撴晬濠婂啠鏀介柍钘�?�閻忋儵鏌曢崱蹇撲壕闂備胶�?椤戝棝骞戦崶褜鍤曞ù鐘�?儛閺佸啴鏌ｉ�?鍥ㄨ吂濠㈣娲栭埞鎴︻敊閺傘倓绶甸梺鍛婃尰濮樸劑骞忚ぐ鎺撴�?�闁绘�?纾崢鐢告⒑閼测斁鎷￠柛鎾�?�懇閵嗗倿鎳�?埡鍐紲濠德板€曢崯顐﹀几濞戙垺鐓曢柍瑙�?劤娴滅偓淇婇悙�?�勨偓鏍暜閹烘柡鍋撳鐓庡⒋鐎殿喖鎲＄�?鐔煎焵椤掑�??钃熼柣鏃傚帶缁犳煡鏌熸�?�瀛樻�?婵炲牜鍘奸埞鎴﹀煡閸℃ぞ绨奸梺鎸庢磸閸ㄨ棄�?�ｆ繝�?�嵆闁靛繒濞€閸炶泛鈹戦悩缁樻�?婵炴潙鍊歌灋闁绘劕妯婂〒濠�?煏閸繃鍣界紒鐘卞嵆閺岋綁�??介銏犱�?濡炪値鍋�?换�?�€骞栭崷�?�熷枂闁告洦鍋嗛敍蹇涙⒒娓氣偓濞佳勭仚闂佺ǹ瀛╅悡锟犲箖濡や降鍋呴柛鎰ㄦ杹閹锋椽姊洪崨濠�?畵閻庢氨鍏樺畷鏇＄疀閺傝绨诲銈嗘尵婵挳宕㈤幘�?�界厽婵炴垵宕弸锔剧磼閻樺�?鈽�?�柍钘�?�槸閳�?�氦绠涙繝鍌欏婵犵數濮电喊宥�?�偂閻�?�粯鐓欐い鎾跺枎缁�?�帡鏌涢�?鈧划�?�囨崲濞戙垹宸濇い鎰╁灮娴煎牆鈹戦�?锋敾婵＄偠妫�?悾鐑筋敃閿曗偓缁�?瀣亜閹邦喖鏋庡ù婊堢畺閺屾盯顢曢敐鍡欘槬缂佺偓鍎崇紞濠囧蓟閳ユ剚鍚�??幖绮光偓鑼埍闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿鏌涘┑鍡楊伂妞ゎ偓绠撻弻娑㈠籍閳ь剟宕归崸妤冨祦闁告劦鍠栭～鍛存煏閸繃鍣�?紒鎰⊕缁绘繈鎮介�?�娴躲垽�?楀闂寸敖婵″弶鍔欏鎾閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒冾潐濞叉﹢銆冮崨杈剧稏婵犲﹤鐗嗛悞�?亜閹哄棗浜惧銈嗘穿缂嶄線銆侀弴銏℃櫇闁逞屽墰缁粯绻濆�?�炰化闂佹悶鍎滈崘�?�堢崜婵＄偑鍊戦崕鑼崲閸繍�?�栨繛�?�簼椤ュ牊绻涢幋鐐跺妞わ絽鎼埞鎴﹀煡閸℃ぞ绨煎銈冨�?�閿曨亪濡存笟鈧�?�€宕煎┑瀣�?闂備礁鎼ú銊╁疮閸ф绠繛�?�簼閻撶喖鏌ｅΟ鍝勫�?闁煎壊浜弻娑㈠棘鐠恒劎鍔悗娈�?枟閹倿鐛€ｎ喗鏅滈柣锝呰�?��?�炴劙�?�虹拠鎻掑毐缂傚秴妫欑粋宥�?�醇閺囩喎浜楅梺绋跨箳閳峰牆鈻撴禒瀣厽闁归偊鍨伴惃铏�?磼閻樺樊鐓奸柡宀�?鍠栭、娆撴偩瀹€鈧悡澶�?⒑閻熸�?�锛嶉柛瀣ㄥ�?栨穱濠囨倻閼恒儲娅嗙紓浣�?☉椤戝�?�骞�?悜鑺モ拻闁�?�本鑹鹃埀顒勵棑缁牊绗熼埀顒勭嵁�?�舵劖鏅搁柣�?�?皺椤斿绻涙潏鍓ф偧缁绢厼鐖煎鎼佸冀椤撶喓鍘搁梺鎼炲劘閸庨亶�?炲ú�?�呯厸濞达絽鎲￠ˉ銏ゆ煛瀹€瀣？濞寸�?�濡�?幏鐘诲�?�閹烘埈娼涚紓鍌�?�?烽懗鑸靛垔椤撱垹鍨傞柛�?�ｆ礀閽冪喖鏌曟繛鐐珕闁稿�?濋弻娑氫�?閸撗�?妲堝銈呴獜閹凤�?,闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿劍�?�?い�?�㈢箰鐓ゆい蹇撳閻�?厼顪�?妶鍡樼闁瑰啿閰ｅ畷婊堝础閻戝棙瀵岄梺闈涙祩閸垳绮堟担璇ユ椽鏁�?崒�?�卞幈闂佺鎻梽鍕磻閿濆鐓欐い鎾跺枎缁楁帒鈹戦垾鐐藉仮闁哄矉绱曢埀顒婄秵閸�?嫰鎮橀幘�?�界厸鐎光偓鐎ｎ剛锛熼梺�?炲苯澧剧紓�?�呮缁傚秹宕奸弴鐔蜂簵闂佺ǹ绻掗埛鍫濃枔娴犲鐓熼柟�?﹀灠閻ㄨ�?�绱掗悩宸吋闁哄瞼鍠栭、娆撴偩瀹€鈧悡澶�?⒑閻熸�?�锛嶉柛瀣ㄥ�?栨穱濠囨倻閼恒儲娅嗙紓浣�?☉椤戝�?�骞�?悜鑺モ拻闁�?�本鑹鹃埀顒勵棑缁牊绗熼埀顒勭嵁閺嶎収鏁冮柨鏇楀亾缁�?儳缍婇弻娑㈩敃閿濆�?�顦ョ紒鐐劤缂嶅﹪�?婚垾鎰佸悑閹肩补鈧尙鐖遍梻浣呵归鍡涘箰閹间緤缍栨繝闈涱儛閺佸棝鏌涚仦鍓ф晼闁靛ň鏅滈埛鎴︽煕濠靛棗�?�い�?�畵閺屾盯�?埀顒勫垂閸ф宓侀柛鎰靛枛椤懘鏌曢崼婵囧櫣缂佹劖绋掔换婵�?偨闂堟刀銏ゆ倵濮橀棿绨芥俊鍙�?��?�瀵挳濮�?閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒冾潐濞叉﹢銆冮崨杈剧稏婵犲﹤鐗嗛悞�?亜閹哄棗浜惧銈嗘穿缂嶄線銆侀弴銏℃櫇闁逞屽墰缁粯绻濆�?�炰化闂佹悶鍎滈崘�?�堢崜婵＄偑鍊戦崕鑼崲閸繍�?�栨繛�?�簼椤ュ牊绻涢幋鐐跺妞わ絽鎼埞鎴﹀煡閸℃ぞ绨煎銈冨�?�閿曨亪濡存笟鈧�?�€宕奸悢鍛婎仧闂備浇娉曢崳锕傚�?閿燂�?
//					if(motor_run_cmd==MOTOR_MODE_STOP)  //motor control
//					{		
//							motor_run_cmd=MOTOR_MODE_START;
//					}	
//						App_MotorControl(motor_run_cmd);	
				//	}
				}	
				else if(rec_Signal==motor_apex_stop_signal) //闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐闁�?�繒鍋涙晶钘壝虹�?涙﹩娈介柣鎰儗閻掍粙鏌熼崣澶�??唉鐎规洜鍠栭、妯衡攦閹傚婵犵數濮�?崑鍡椼€掓繝姘�?闁割偅绻�?ˉ婊呯磼濡や�?�鍝�?柡宀�?�?楠炴瑩宕熼鐘垫闂備胶�?笟妤呭窗濞戞氨涓嶆繛鎴炃氬Σ�?熺�?�閸℃绠氶柛瀣尰缁绘繂�?濋鐘插箻闂備浇顕栭崢鐣屾暜閹烘绀夋繝濠傚�?缁犻箖鏌涘☉鍗炰簻闁诲繐�?堕幈銊︾節閸愨斂浠㈤悗瑙勬处閸�?﹤鐣烽悢纰辨晝闁绘�?�娓规竟鏇㈡⒑閸撴彃浜濇繛鍙夌墱婢�?�洝銇愰幒鎾跺幐閻庡箍鍎�?鍛婄閹扮増鐓曢悗锝庡亝鐏忣厽銇勯锝囩疄妞ゃ垺顨婂畷鎺戔攦閻愵亜�?傛慨濠冩そ瀹曨偊宕熼鍛晧闂備�?�鎲″褰掑垂閸ф宓侀柛鎰靛枛椤懘鏌曢崼婵囧櫣缂佹劖绋掔换婵�?偨闂堟刀銏ゆ倵濮橀棿绨芥俊鍙�?��?�瀵挳濮�?閳锯偓閹风粯绻涙潏鍓ф偧闁稿簺鍊濆畷鐢稿焵椤掑�?鈷戦柟鑲╁仜閳ь剚鐗犻幃�?�鎮╅懡銈呯ウ闂婎偄娲︾粙鎺楀疾閹间焦鐓ラ柣鏇炲€�?�?氾拷
				{
					apexMotorStartFlag=TRUE;//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿鏌涘┑鍡楊伂妞ゎ偓绠撻弻娑㈠籍閳ь剟宕归崸妤冨祦闁告劦鍠栭～鍛存煏閸繃顥滈柍褜鍓涚划顖涚┍婵犲浂鏁嶆慨�?�嗗�?閸�?�垰鈹戦悙鑼ⅵ缂佺姵鐗曢～蹇撁洪鍛闂侀潧鐗嗛幉娑㈠焺閸愵亞顔曢梺鍝勵槹閸ㄥ爼宕板鈧弻銊モ攽閸繀娌悗鍨緲鐎氼喗绂掗敂鍓х煓濠㈠墎枪椤ユ碍绻濆▓�?灍闁挎洍鏅犲畷婊冣槈濮樿京鐓�??┑鐐叉▕娴滄繈鎮″☉銏″€堕柣鎰邦杺閸ゆ瑥鈹戦鐓庘偓鍧�?�蓟閻旂⒈鏁婇柛婵嗗娴煎牓鎮楃憴鍕８闁告柨绉堕幑銏�?攽鐎ｎ亞顦板銈�?箰濡盯鏁嶉悢鍏尖拻闁�?�本鑹鹃埀顒勵棑缁牊鎷呴搹鍦槸婵犵數濮村▔姘緞閹邦剛顔掗柣鐘叉穿鐏忔瑩藝瑜嶉埞鎴︻敊婵劒绮堕梺绋�?�儐閹搁箖鍩�?椤掍緡鍟忛柛锝庡櫍瀹曟垶绻濋崶鈺佺ウ濠碘�?�鍨伴崥瀣偓�?�哺閺屾稑鈻庤箛锝嗏枔濠�?��?�鍋嗛崰鏍ь潖缂佹鐟归柍褜鍓欓…鍥樄闁�?�啫鍥у耿婵＄偑鍨虹粙鎴ｇ亙闂佸憡绮堥悞锕傚疾濠婂牊鈷戦柛鎾村絻娴滄繃绻涢崣澶涜�?块柛鈺傜洴�?�炲鏁傞挊澶�?��?�闂備浇顕栭崢鐣屾暜閹烘挷绻嗗┑鍌氭啞閻撴盯�?楅敐搴′簻闁�?�繑鎸抽弻鈥崇暆鐎ｎ剛锛熼梺�?炲苯澧剧紓�?�呮缁傚秹宕奸弴鐔蜂簵闂佺ǹ绻掗埛鍫濃枔娴犲鐓熼柟�?﹀灠閻ㄨ�?�绱掗悩宸吋闁哄瞼鍠栭、娆撴偩瀹€鈧悡澶�?⒑閻熸�?�锛嶉柛瀣ㄥ�?栨穱濠囨倻閼恒儲娅嗙紓浣�?☉椤戝�?�骞�?悜鑺モ拻闁�?�本鑹鹃埀顒勵棑缁牊绗熼埀顒勭嵁閺嶎収鏁冮柨鏇楀亾缁�?儳缍婇弻娑㈩敃閿濆�?�顦ョ紒鐐劤缂嶅﹪�?婚垾鎰佸悑閹肩补鈧尙鐖遍梻浣呵归鍡涘箰閹间緤缍栨繝闈涱儐閸婇攱銇�?幒鍡椾�?�濡�?倕�?�忛幏锟�?	
					if(motor_run_cmd==MOTOR_MODE_START) //motor control
					{						
						motor_run_cmd=MOTOR_MODE_STOP;
					}		
					menuPage=MENU_HOME_PAGE;		//page change					
					MenuPageTurns(menuPage);
					App_MotorControl(MOTOR_MODE_STOP);
				}					
				else
				{							
					WorkPageFlash(MENU_APEX_AND_MOTOR_PAGE,xTaskGetTickCount());
					if(sys_param_un.device_param.apexFunctionLoad==0)//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?節瀹曞爼鍩℃担鍦簴缂傚倷鑳舵繛鈧紒鐘崇墵瀵鈽夐�?�鐘栥劑鏌曡箛濠傚⒉闁绘繃鐗�?�穱濠囨倷妫版繃缍堟繝鐢靛仜閿曨�?鐛崘銊ф殝闁逛絻娅曢悗濠�?椤愩垺澶勯柟灏栨櫆缁傛帡鎮℃惔顔藉瘜闂侀潧鐗嗛幊鎰不娴煎瓨鐓熼柡�?�庡亜鐢埖銇勯銏㈢闁圭厧婀遍幉鎾礋椤愩倧绱￠梻鍌欑窔濞佳勭仚闂佺ǹ瀛╅悡锟犲箖濡や降鍋呴柛鎰ㄦ杹閹锋椽姊洪崨濠�?畵閻庢氨鍏樺畷鏇＄疀閺傝绨诲銈嗘尵婵挳宕㈤幘�?�界厽婵炴垵宕弸锔剧磼閻樺�?鈽�?�柍钘�?�槸閳�?�氦绠涙繝鍌欏婵犵數濮电喊宥�?�偂閻�?�粯鐓欐い鎾跺枎缁�?�帡鏌涢�?鈧划�?�囨崲濞戙垹宸濇い鎰╁灮娴煎牆鈹戦�?锋敾婵＄偠妫�?悾鐑筋敃閿曗偓缁�?瀣亜閹邦喖鏋庡ù婊堢畺閺屾盯顢曢敐鍡欘槬缂佺偓鍎崇紞濠囧蓟閳ユ剚鍚�??幖绮光偓鑼埍闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿鏌涘┑鍡楊伂妞ゎ偓绠撻弻娑㈠籍閳ь剟宕归崸妤冨祦闁告劦鍠栭～鍛存煥濞戞ê顏柟铏箓椤啴濡堕崱娆忣潷闂佽崵鍟欓崗鐐洴椤㈡�?�?欑划瑙勫濠电偠鎻徊鍧楀箠閹�?椿鏁�?繝濠傛噽绾惧ジ鎮归崶銊ョ祷闁逞屽�?椤兘鐛幋锕�?�?呴柕�?濇噽椤撴椽�?�虹紒�?�堜緵闁稿瀚粋�?��?�倷椤掍�?��??�?┑�?�筋殔濡鏅堕幍�?�剧＜濠㈣泛顑嗛妵婵�?煛瀹€瀣？濞寸�?�濡�?幏鐘诲�?�閹烘繃缍嗛梻鍌欐祰椤曟牠宕伴幘璇插�?鐟滅増甯楅崑鈺呮煥閻斿搫校闁绘帊�?欓弻銊╁籍閳ь剟�??搁懡銈嗩潟闁�?儤�?�荤壕浠�?煕鐏炴崘澹橀柍�?�鍓涢崗姗€骞冮悙鐑樻櫆闁告挆鍛幆闂備礁鎼粙渚€宕ｈ箛鏇犵闁哄诞宀�?鍞甸柣鐘叉惈閹碱偊藝椤掑�?鐓曢悗锝庡亝鐏忣厽銇�?锝囩疄妞ゃ垺顨婂畷鎺戔攦閻愵亜�?傛慨濠冩そ瀹曨偊宕熼鍛晧闂備�?�鎲″褰掑垂閸ф宓侀柛鎰靛枛椤懘鏌曢崼婵囧櫣缂佹劖绋掔换婵�?偨闂堟刀銏ゆ倵濮橀棿绨芥俊鍙�?��?�瀵挳濮�?閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒冾潐濞叉﹢銆冮崨杈剧稏婵犲﹤鐗嗛悞�?亜閹哄棗浜惧銈嗘穿缂嶄線銆侀弴銏℃櫇闁逞屽墰缁粯绻濆�?�炰化闂佹悶鍎滈崘�?�堢崜婵＄偑鍊戦崕鑼崲閸繍�?�栨繛�?�簼椤ュ牊绻涢幋鐐跺妞わ絽鎼埞鎴﹀煡閸℃ぞ绨煎銈冨�?�閿曨亪濡存笟鈧�?�€宕煎┑瀣�?闂備礁鎼ú銊╁窗閹捐鍌ㄩ柦�?�?槴閺�?浠�??煃閽樺顥滈柣蹇曞█閺岋綀绠涢�?鍌滅杽闂佽鍣换婵�?嵁閸�?剚鍋�?柛�?�犲灩鐢鏌ｉ悢鍝ョ煁缂侇喗鎸搁悾宄扳�?閸惊鈺�?煃鏉炴�?�鍏屾い鏂挎濮婅�?�鎹�?妸銉︾彚闂佺懓鍤�?幏锟�?
					{	
						apexMotorStartFlag=TRUE;//闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿鏌涘┑鍡楊伂妞ゎ偓绠撻弻娑㈠籍閳ь剟宕归崸妤冨祦闁告劦鍠栭～鍛存煏閸繃顥滈柍褜鍓涚划顖涚┍婵犲浂鏁嶆慨�?�嗗�?閸�?�垰鈹戦悙鑼ⅵ缂佺姵鐗曢～蹇撁洪鍛闂侀潧鐗嗛幉娑㈠焺閸愵亞顔曢梺鍝勵槹閸ㄥ爼宕板鈧弻銊モ攽閸繀娌悗鍨緲鐎氼喗绂掗敂鍓х煓濠㈠墎枪椤ユ碍绻濆▓�?灍闁挎洍鏅犲畷婊冣槈濮樿京鐓�??┑鐐叉▕娴滄繈鎮″☉銏″€堕柣鎰邦杺閸ゆ瑥鈹戦鐓庘偓鍧�?�蓟閻旂⒈鏁婇柛婵嗗娴煎牓鎮楃憴鍕８闁告柨绉堕幑銏�?攽鐎ｎ亞顦板銈�?箰濡盯鏁嶉悢鍏尖拻闁�?�本鑹鹃埀顒勵棑缁牊鎷呴搹鍦槸婵犵數濮村▔姘緞閹邦剛顔掗柣鐘叉穿鐏忔瑩藝瑜嶉埞鎴︻敊婵劒绮堕梺绋�?�儐閹搁箖鍩�?椤掍緡鍟忛柛锝庡櫍瀹曟垶绻濋崶鈺佺ウ濠碘�?�鍨伴崥瀣偓�?�哺閺屾稑鈻庤箛锝嗏枔濠�?��?�鍋嗛崰鏍ь潖缂佹鐟归柍褜鍓欓…鍥樄闁�?�啫鍥у耿婵＄偑鍨虹粙鎴ｇ亙闂佸憡绮堥悞锕傚疾濠婂牊鈷戦柛鎾村絻娴滄繃绻涢崣澶涜�?块柛鈺傜洴�?�炲鏁傞挊澶�?��?�闂備浇顕栭崢鐣屾暜閹烘挷绻嗗┑鍌氭啞閻撴盯�?楅敐搴′簻闁�?�繑鎸抽弻鈥崇暆鐎ｎ剛锛熼梺�?炲苯澧剧紓�?�呮缁傚秹宕奸弴鐔蜂簵闂佺ǹ绻掗埛鍫濃枔娴犲鐓熼柟�?﹀灠閻ㄨ�?�绱掗悩宸吋闁哄瞼鍠栭、娆撴偩瀹€鈧悡澶�?⒑閻熸�?�锛嶉柛瀣ㄥ�?栨穱濠囨倻閼恒儲娅嗙紓浣�?☉椤戝�?�骞�?悜鑺モ拻闁�?�本鑹鹃埀顒勵棑缁牊绗熼埀顒勭嵁閺嶎収鏁冮柨鏇楀亾缁�?儳缍婇弻娑㈩敃閿濆�?�顦ョ紒鐐劤缂嶅﹪�?婚垾鎰佸悑閹肩补鈧尙鐖遍梻浣呵归鍡涘箰閹间緤缍栨繝闈涱儐閸婇攱銇�?幒鍡椾�?�濡�?倕�?�忛幏锟�?							
						motor_run_cmd =	MOTOR_MODE_STOP;						
						App_MotorControl(motor_run_cmd);							
						menuPage=MENU_HOME_PAGE;							
						MenuPageTurns(menuPage);
					}							
				}    			
			break;	
			case 	MENU_CHARGING_PAGE:				
				if(motor_run_cmd==MOTOR_MODE_START) //motor control
				{						
					motor_run_cmd=MOTOR_MODE_STOP;
					App_MotorControl(motor_run_cmd);						
				}					
			break;	
			case 	MENU_MAX_PAGE_NUM:				
				if(rec_Signal==run_button_long_press_signal)
				{									
					#ifdef WDT_ENABLE//	1s								
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
					vTaskDelay(MAX_WDT_FEED_TIME_MS);
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
					vTaskDelay(MAX_WDT_FEED_TIME_MS);						
					#else
					vTaskDelay(600);//	1s
					#endif							
					MenuDevicePowerOff(1);													
				}		
				else //if(rec_Signal==power_off_signal)
				{
					#ifdef WDT_ENABLE//	1s								
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
					vTaskDelay(MAX_WDT_FEED_TIME_MS);
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
					vTaskDelay(MAX_WDT_FEED_TIME_MS);						
					#else
					vTaskDelay(600);//	1s
					#endif	
					MenuDevicePowerOff(0);
				}		
			break;
			default:
				if(motor_run_cmd==MOTOR_MODE_START) //motor control
				{						
					motor_run_cmd=MOTOR_MODE_STOP;								
				}
				App_MotorControl(motor_run_cmd);
				menuPage=MENU_HOME_PAGE;							
				MenuPageTurns(menuPage);
			break;			
		}		 
		if(rec_Signal!=null_signal)
		{	
			#ifdef DEBUG_RTT
				SEGGER_RTT_printf(0, "key run %d\r\n", rec_Signal);
			#endif
			rec_Signal= null_signal;	
			if(menuPage==MENU_SYSTEM_SET_PAGE||menuPage	==MENU_HOME_PAGE)
			{
				xSemaphoreGive(xSemaphoreDispRfresh); //闂傚倸鍊搁崐鎼佸磹閹间�?�纾�?�?瑰�??鍣�?�ぐ鎺戠倞鐟滃�?搁弽顓熺厸闁搞儯鍎遍悘鈺�?磼閹邦収娈滈柡宀�?鍠栭幃�?�宕奸悢鍝勫殥闂備�?�鎲￠崹瑙勬叏閹绢喖鐓橀柟杈鹃�??閸�??劙鎮楅崷顓炐㈡い銉︾箘缁辨挻鎷呴�?鍕�?�偓宀�?煕閵娿儳鍩ｇ�?殿喖�?烽弫鎰緞婵犲�?�鍚呴梻浣瑰缁诲倿骞夊☉銏犵缂備焦�?�?崢鎼佹⒑閸涘﹣绶遍柛鐘愁殜瀹曟劙骞�?悧鍫㈠幐閻庡厜鍋撻悗锝庡墰閻﹀牓鎮楃憴鍕閻㈩垱�?熼悘鎺�?�煟閻愬�?鈻撻柍�?�鍓欓崢鏍ㄧ珶閺囥垺鈷戦柛婵嗗閳�?�鏌涚�?Ｑ冧壕闂備胶�?椤戝棝骞愰幖渚婄稏婵犻潧顑愰�?鍡涙煕鐏炲墽鏁栭柕濞�?櫆閳锋垿鏌涘┑鍡楊伂妞ゎ偓绠撻弻娑㈠籍閳ь剟宕归崸妤冨祦闁告劦鍠栭～鍛存煏閸繃鍣�?紒鎰⊕缁绘繈鎮介�?�娴躲垽�?楀闂寸敖婵″弶鍔欏鎾閳锯偓閹风粯绻涙潏鍓у埌闁硅姤�?庣划鏃堟倻濡晲绨�?�梺闈涱檧闂�?�?鐣峰畝鈧埀顒冾潐濞叉﹢銆冮崨杈剧稏婵犲﹤鐗嗛悞�?亜閹哄棗浜惧銈嗘穿缂嶄線銆侀弴銏℃櫇闁逞屽墰缁鎳￠妶鍥╋紳婵炶�?缍€椤曟牠鎮炴�?�瀣厱婵炲�?�绻愰顓㈡煛瀹€瀣М濠�?�喒鍋撻梺瀹犳〃閼宠埖瀵奸崶�?�佲拺闁告縿鍎辨牎濠殿喗菧閸斿酣宕氶幒妤�?�?嶉柛顐ｇ箞閸炲爼姊洪�?鍕窛闁哥姵鎹囧畷銏ゅ垂椤愶紕绠氶梺缁樺姇濡﹪宕曡箛鎾�?棃鎮╅搹顐⑩偓鎰偓瑙勬磸閸ㄤ粙鐛鈧、娆撴偩鐏炶棄绗氶梺鑽ゅ枑缁秶鍒掗幘宕囨�?�婵犲﹤鍠氬鈺傘亜閹烘埈妲归柛宥囨�?�?婃椽�?�?ù銉ョ墦瀵彃饪伴崼婵堬紱闂佺ǹ鐬奸崑鐐烘偂閵夆晜鐓熼柡鍌涘閹牏鈧稒绻勭槐鎾存�?�閹绘帊澹曢梻浣虹《閸撴繄�?欓幒妤佸亗闁绘柨鍚�??悡娆撳级閸繂鈷旈柛鎺撳閹喖�?濋懜纰樻嫼闂佸憡绻傜�?氼參宕冲ú顏呯厵�?�ゆ梻鍘ч埀顒€鐏濋锝嗙節�?橆厽娅㈤梺缁樓圭亸娆撴晬濠婂啠鏀介柍钘�?�閻忋儵鏌曢崱蹇撲壕闂備胶�?椤戝棝骞戦崶褜鍤曞ù鐘�?儛閺佸啴鏌ｉ�?鍥ㄨ吂濠㈣娲栭埞鎴︻敊缁涘鍔烽梺绋�?�儐閹搁箖鍩�?椤掍胶鍟查柟鍑ゆ�?
			}	
		}
		else
		{			
			if(xQueueReceive(xQueueMenuValue,&menuPage,0) == pdTRUE)
			{			
				MenuPageTurns(menuPage);				
			}			
			if(xQueueReceive(xQueueBatValue, &batDispValue, 0) == pdTRUE)
			{	
				if(batDispValue!=0)
				{
					if(menuPage==MENU_HOME_PAGE||menuPage==MENU_ONLY_APEX_PAGE)
					{
						disp_batValue(2,32,batDispValue);						
					}
					else if(menuPage==MENU_CHARGING_PAGE)
					{
						pMalloc=pvPortMalloc(256);
						if(pMalloc!=NULL)
						{
							OLED_Display_CHARGING(batDispValue,1);		
							vPortFree(pMalloc);
						}						
					}
				}					
				xSemaphoreGive(xSemaphoreDispRfresh);
			}
		}				
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
		#endif
		 vTaskDelay(2);//1ms
	}
}
