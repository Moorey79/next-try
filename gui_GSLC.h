
#ifndef _GUISLICE_GEN_H
#define _GUISLICE_GEN_H

// ------------------------------------------------// Headers to include
// ------------------------------------------------
#include "GUIslice.h"
#include "GUIslice_drv.h"

// Include any extended elements
//<Includes !Start!>
// Include extended elements
#include "elem/XProgress.h"
#include "elem/XSeekbar.h"
#include "elem/XTextbox.h"
#include "elem/XTogglebtn.h"
//<Includes !End!>

// ------------------------------------------------
// Headers and Defines for fonts
// Note that font files are located within the Adafruit-GFX library folder:
// ------------------------------------------------
//<Fonts !Start!>
#if defined(DRV_DISP_TFT_ESPI)
  #error E_PROJECT_OPTIONS tab->Graphics Library should be TFT_eSPI
#endif
#include <Adafruit_GFX.h>
#include "Fonts/FreeSansBoldOblique24pt7b.h"
#include "Fonts/FreeSansBoldOblique9pt7b.h"
#include "NotoLatin1_8pt.h"
//<Fonts !End!>

// ------------------------------------------------
// Defines for resources
// ------------------------------------------------
//<Resources !Start!>
//<Resources !End!>

// ------------------------------------------------
// Enumerations for pages, elements, fonts, images
// ------------------------------------------------
//<Enum !Start!>
enum {E_PG_BASE,E_PG_MAIN,SetPoints,Running,Finished};
enum {E_ELEM_ZERO,E_ELEM_BTN4,E_ELEM_BTN7,E_ELEM_BTN8,E_ELEM_PROGRESS2,E_ELEM_SEEKBAR7
      ,E_ELEM_TEXT11,E_ELEM_TEXT12,E_ELEM_TEXT13,E_ELEM_TEXT14
      ,E_ELEM_TEXT19,E_ELEM_TEXT21,E_ELEM_TEXT26,E_ELEM_TEXT9
      ,E_ELEM_TEXTBOX5,E_ELEM_TEXTBOX6,E_ELEM_TEXTBOX7,E_ELEM_TEXTrun
      ,E_ELEM_TEXTtemp,E_ELEM_Time,E_ELEM_rem,E_ELEM_ttg,E_Heat_Seek
      ,E_Heatstat,E_Heattog,E_PumpStat,E_Pump_Seek,E_Pumptog,E_StartBtn
      ,E_StartBtn2,E_Stop_Seek,HeatOFF,PumpOFF,PumpON,E_ELEM_RESET_BTN,E_ELEM_MAX};
enum {GSLC_GROUP_ID_heat,GSLC_GROUP_ID_pump};
// Must use separate enum for fonts with MAX_FONT at end to use gslc_FontSet.
enum {E_BUILTIN10X16,E_BUILTIN15X24,E_BUILTIN25X40,E_BUILTIN5X8
      ,E_FREESANSBOLDOBLIQUE24,E_FREESANSBOLDOBLIQUE9,E_NOTOLATIN1_8PT
      ,MAX_FONT};
//<Enum !End!>

// ------------------------------------------------
// Instantiate the GUI
// ------------------------------------------------

// ------------------------------------------------
// Define the maximum number of elements and pages
// ------------------------------------------------
//<ElementDefines !Start!>
#define MAX_PAGE                5

#define MAX_ELEM_PG_BASE 0 // # Elems total on page
#define MAX_ELEM_PG_BASE_RAM MAX_ELEM_PG_BASE // # Elems in RAM

#define MAX_ELEM_PG_MAIN 2 // # Elems total on page
#define MAX_ELEM_PG_MAIN_RAM MAX_ELEM_PG_MAIN // # Elems in RAM

#define MAX_ELEM_SetPoints 18 // # Elems total on page
#define MAX_ELEM_SetPoints_RAM MAX_ELEM_SetPoints // # Elems in RAM

#define MAX_ELEM_Running 15 // # Elems total on page
#define MAX_ELEM_Running_RAM MAX_ELEM_Running // # Elems in RAM

#define MAX_ELEM_Finished 2 // # Elems total on page
#define MAX_ELEM_Finished_RAM MAX_ELEM_Finished // # Elems in RAM
//<ElementDefines !End!>

// ------------------------------------------------
// Create element storage
// ------------------------------------------------
gslc_tsGui                      m_gui;
gslc_tsDriver                   m_drv;
gslc_tsFont                     m_asFont[MAX_FONT];
gslc_tsPage                     m_asPage[MAX_PAGE];

//<GUI_Extra_Elements !Start!>
gslc_tsElem                     m_asBasePage1Elem[MAX_ELEM_PG_BASE_RAM];
gslc_tsElemRef                  m_asBasePage1ElemRef[MAX_ELEM_PG_BASE];
gslc_tsElem                     m_asPage1Elem[MAX_ELEM_PG_MAIN_RAM];
gslc_tsElemRef                  m_asPage1ElemRef[MAX_ELEM_PG_MAIN];
gslc_tsElem                     m_asPage3Elem[MAX_ELEM_SetPoints_RAM];
gslc_tsElemRef                  m_asPage3ElemRef[MAX_ELEM_SetPoints];
gslc_tsElem                     m_asPage4Elem[MAX_ELEM_Running_RAM];
gslc_tsElemRef                  m_asPage4ElemRef[MAX_ELEM_Running];
gslc_tsElem                     m_asPage5Elem[MAX_ELEM_Finished_RAM];
gslc_tsElemRef                  m_asPage5ElemRef[MAX_ELEM_Finished];
gslc_tsXTextbox                 m_sTextbox5;
char                            m_acTextboxBuf5[168]; // NRows=6 NCols=28
gslc_tsXTextbox                 m_sTextbox6;
char                            m_acTextboxBuf6[168]; // NRows=6 NCols=28
gslc_tsXTextbox                 m_sTextbox7;
char                            m_acTextboxBuf7[168]; // NRows=6 NCols=28
gslc_tsXSeekbar                 m_sXSeekbar5;
gslc_tsXSeekbar                 m_sXSeekbar6;
gslc_tsXSeekbar                 m_sXSeekbar7;
gslc_tsXTogglebtn               m_asXToggle1;
gslc_tsXTogglebtn               m_asXToggle3;
gslc_tsXProgress                m_sXBarGauge2;

#define MAX_STR                 100

//<GUI_Extra_Elements !End!>

// ------------------------------------------------
// Program Globals
// ------------------------------------------------

// Element References for direct access
//<Extern_References !Start!>
extern gslc_tsElemRef* HeaterSTOPslide;
extern gslc_tsElemRef* HeaterSTOPval;
extern gslc_tsElemRef* PumpSTARTslide;
extern gslc_tsElemRef* PumpSTARTval;
extern gslc_tsElemRef* PumpSTOPslide;
extern gslc_tsElemRef* PumpSTOPval;
extern gslc_tsElemRef* m_HeatStat6;
extern gslc_tsElemRef* m_PumpStat6;
extern gslc_tsElemRef* m_heattog;
extern gslc_tsElemRef* m_pElemBtn1;
extern gslc_tsElemRef* m_pElemBtn1_4;
extern gslc_tsElemRef* m_pElemOutTime;
extern gslc_tsElemRef* m_pElemOutTxt20;
extern gslc_tsElemRef* m_pElemProgress2;
extern gslc_tsElemRef* m_pElemTextbox2_3_6;
extern gslc_tsElemRef* m_pElemTextbox2_4_7;
extern gslc_tsElemRef* m_pElemTextbox2_5;
extern gslc_tsElemRef* m_pElem_ttg;
extern gslc_tsElemRef* m_pTextSlider2;
extern gslc_tsElemRef* m_pTextSlider2;
extern gslc_tsElemRef* m_pTextSlider2;
extern gslc_tsElemRef* m_pumptog;
extern gslc_tsElemRef* runBACKbtn;
extern gslc_tsElemRef* runSTOPbtn;
extern gslc_tsElemRef* spBACKbtn;

// Declare E_RESET_RUNS_BTN as a pointer.
//gslc_tsElemRef* E_RESET_RUNS_BTN;

// Define gslc_style_none
	typedef struct {
		uint8_t  fontId;
		uint16_t col;
		uint16_t col_hi;
		uint8_t  alignH;
		uint8_t  alignV;
	} gslc_tsStyle;
	gslc_tsStyle gslc_style_none = {0, 0, 0, 0, 0};

// Function declarations (forward declarations) for functions used in this header.
bool CbResetRunsBtn(void* pvGui, void* pvElemRef, gslc_teTouch eTouch, int16_t nX, int16_t nY);

// ------------------------------------------------
// Define debug message function
static int16_t DebugOut(char ch);

// ------------------------------------------------
// Callback Methods
// ------------------------------------------------
bool CbBtnCommon(void* pvGui,void *pvElemRef,gslc_teTouch eTouch,int16_t nX,int16_t nY);
bool CbCheckbox(void* pvGui, void* pvElemRef, int16_t nSelId, bool bState);
bool CbDrawScanner(void* pvGui,void* pvElemRef,gslc_teRedrawType eRedraw);
bool CbKeypad(void* pvGui, void *pvElemRef, int16_t nState, void* pvData);
bool CbListbox(void* pvGui, void* pvElemRef, int16_t nSelId);
bool CbSlidePos(void* pvGui,void* pvElemRef,int16_t nPos);
bool CbSpinner(void* pvGui, void *pvElemRef, int16_t nState, void* pvData);
bool CbTickScanner(void* pvGui,void* pvScope);

// ------------------------------------------------
// Create page elements
// ------------------------------------------------
void InitGUIslice_gen()
{
  gslc_tsElemRef* pElemRef = NULL;

  if (!gslc_Init(&m_gui,&m_drv,m_asPage,MAX_PAGE,m_asFont,MAX_FONT)) { return; }

  // ------------------------------------------------
  // Load Fonts
  // ------------------------------------------------
//<Load_Fonts !Start!>
    if (!gslc_FontSet(&m_gui,E_BUILTIN10X16,GSLC_FONTREF_PTR,NULL,2)) { return; }
    if (!gslc_FontSet(&m_gui,E_BUILTIN15X24,GSLC_FONTREF_PTR,NULL,3)) { return; }
    if (!gslc_FontSet(&m_gui,E_BUILTIN25X40,GSLC_FONTREF_PTR,NULL,5)) { return; }
    if (!gslc_FontSet(&m_gui,E_BUILTIN5X8,GSLC_FONTREF_PTR,NULL,1)) { return; }
    if (!gslc_FontSet(&m_gui,E_FREESANSBOLDOBLIQUE24,GSLC_FONTREF_PTR,&FreeSansBoldOblique24pt7b,1)) { return; }
    if (!gslc_FontSet(&m_gui,E_FREESANSBOLDOBLIQUE9,GSLC_FONTREF_PTR,&FreeSansBoldOblique9pt7b,1)) { return; }
    if (!gslc_FontSet(&m_gui,E_NOTOLATIN1_8PT,GSLC_FONTREF_PTR,&NotoLatin1_8pt,1)) { return; }
//<Load_Fonts !End!>

//<InitGUI !Start!>
  gslc_PageAdd(&m_gui,E_PG_BASE,m_asBasePage1Elem,MAX_ELEM_PG_BASE_RAM,m_asBasePage1ElemRef,MAX_ELEM_PG_BASE);
  gslc_PageAdd(&m_gui,E_PG_MAIN,m_asPage1Elem,MAX_ELEM_PG_MAIN_RAM,m_asPage1ElemRef,MAX_ELEM_PG_MAIN);
  gslc_PageAdd(&m_gui,SetPoints,m_asPage3Elem,MAX_ELEM_SetPoints_RAM,m_asPage3ElemRef,MAX_ELEM_SetPoints);
  gslc_PageAdd(&m_gui,Running,m_asPage4Elem,MAX_ELEM_Running_RAM,m_asPage4ElemRef,MAX_ELEM_Running);
  gslc_PageAdd(&m_gui,Finished,m_asPage5Elem,MAX_ELEM_Finished_RAM,m_asPage5ElemRef,MAX_ELEM_Finished);

  // Now mark E_PG_BASE as a "base" page which means that it's elements
  // are always visible. This is useful for common page elements.
  gslc_SetPageBase(&m_gui, E_PG_BASE);


  // NOTE: The current page defaults to the first page added. Here we explicitly
  //       ensure that the main page is the correct page no matter the add order.
  gslc_SetPageCur(&m_gui,E_PG_MAIN);
  
  // Set Background to a flat color
  gslc_SetBkgndColor(&m_gui,GSLC_COL_BLACK);

  // -----------------------------------
  // PAGE: E_PG_BASE
  

  // -----------------------------------
  // PAGE: E_PG_MAIN
  
  
  // create E_StartBtn2 button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_StartBtn2,E_PG_MAIN,
    (gslc_tsRect){16,33,256,69},(char*)"Start",0,E_FREESANSBOLDOBLIQUE24,&CbBtnCommon);
	gslc_ElemSetTxtMargin(&m_gui,pElemRef,10);
	gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BROWN,GSLC_COL_BLUE_DK4,GSLC_COL_CYAN);
	m_pElemBtn1 = pElemRef;
  
  // create E_ELEM_BTN4 button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN4,E_PG_MAIN,
    (gslc_tsRect){16,123,256,69},(char*)"Setpoints",0,E_FREESANSBOLDOBLIQUE24,&CbBtnCommon);
  gslc_ElemSetTxtMargin(&m_gui,pElemRef,10);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BROWN,GSLC_COL_BLUE_LT4,GSLC_COL_CYAN);
  gslc_ElemSetRoundEn(&m_gui, pElemRef, true);
  m_pElemBtn1_4 = pElemRef;

  // -----------------------------------
  // PAGE: SetPoints
  //
  // Create hidden button for resetting runs
   pElemRef = gslc_ElemCreateBtnTxt( &m_gui, E_ELEM_RESET_BTN, SetPoints, (gslc_tsRect){5, 200, 20, 20}, "", 0, E_BUILTIN5X8, &CbResetRunsBtn);
  //gslc_tsElemRef = gslc_ElemCreateBtnTxt(&m_gui, E_RESET_RUNS_BTN, SetPoints, (gslc_tsRect) {5, 200, 20, 20,0});
 
  
	gslc_ElemSetFrameEn(&m_gui, pElemRef, false);
	gslc_ElemSetTxtCol(&m_gui, pElemRef, GSLC_COL_BLACK);
  gslc_ElemSetCol(&m_gui, pElemRef, GSLC_COL_BLACK, GSLC_COL_BLACK, GSLC_COL_BLACK);
	(gslc_tsRect){133 ,115, 170, 30}, E_BUILTIN5X8,
	(char*)&m_acTextboxBuf5, 6, 28;
	gslc_ElemXTextboxWrapSet(&m_gui,pElemRef,false);
	gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_YELLOW);
	gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_GRAY_DK3,GSLC_COL_BLACK,GSLC_COL_BLACK);
	m_pElemTextbox2_5 = pElemRef;

  // Create E_ELEM_TEXT9 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT9,SetPoints,(gslc_tsRect) {136,117,99,17},
    (char*)"Heater stop",0,E_FREESANSBOLDOBLIQUE9);

  // Create textbox
  pElemRef = gslc_ElemXTextboxCreate(&m_gui, E_ELEM_TEXTBOX6, SetPoints, &m_sTextbox6,
	(gslc_tsRect){134, 177, 170, 30}, E_BUILTIN5X8,	(char*)&m_acTextboxBuf6, 6, 28);
  gslc_ElemXTextboxWrapSet(&m_gui, pElemRef, false);
  gslc_ElemSetTxtCol(&m_gui, pElemRef, GSLC_COL_BLUE);
  gslc_ElemSetCol(&m_gui, pElemRef, GSLC_COL_GRAY_DK3, GSLC_COL_BLACK, GSLC_COL_BLACK);
  m_pElemTextbox2_3_6 = pElemRef;
  
	// Create E_Stop_Seek text label
	pElemRef = gslc_ElemCreateTxt(&m_gui, E_Stop_Seek, SetPoints,(gslc_tsRect){137, 180, 95, 17},
	(char*)"Pump Stop", 0, E_FREESANSBOLDOBLIQUE9);

   // Create textbox
  gslc_tsElemRef* gslc_ElemXTextboxCreate(gslc_tsGui* pGui, int16_t nElemId, int16_t nPage, gslc_tsXTextbox* pXTextbox, gslc_tsRect rElem, int16_t nFontId, char* pStrBuf, uint8_t nStrBufMax, int16_t nStrLenMax);

	gslc_ElemXTextboxWrapSet(&m_gui, pElemRef, false);
	gslc_ElemSetTxtCol(&m_gui, pElemRef, GSLC_COL_YELLOW);
	gslc_ElemSetCol(&m_gui, pElemRef, GSLC_COL_GRAY_DK3, GSLC_COL_BLACK, GSLC_COL_BLACK);
  m_pElemTextbox2_4_7 = pElemRef;

  // Create E_ELEM_TEXT11 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT11,SetPoints,(gslc_tsRect){137,48,97,17},
    (char*)"Pump Start",0,E_FREESANSBOLDOBLIQUE9);

  // Create E_ELEM_TEXT12 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT12,SetPoints,(gslc_tsRect){270,48,20,10},
    (char*)"\xba",0,E_NOTOLATIN1_8PT);

  // Create E_ELEM_TEXT13 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT13,SetPoints,(gslc_tsRect){280,117,20,10},//  if limiting to 2 digits, change 280 back to 270
    (char*)"\xba",0,E_NOTOLATIN1_8PT);

  // Create E_ELEM_TEXT14 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT14,SetPoints,(gslc_tsRect){270,180,20,10},
    (char*)"\xba",0,E_NOTOLATIN1_8PT);

  // Create seekbar E_Pump_Seek
  pElemRef = gslc_ElemXSeekbarCreate(&m_gui,E_Pump_Seek,SetPoints,&m_sXSeekbar5,
    (gslc_tsRect){127,22,160,20},40,80,60,
    4,2,8,GSLC_COL_BLUE,GSLC_COL_GRAY,GSLC_COL_BLUE_DK2,false);
  gslc_ElemXSeekbarSetPosFunc(&m_gui,pElemRef,&CbSlidePos);
  gslc_ElemXSeekbarSetStyle(&m_gui,pElemRef,true,GSLC_COL_BLUE_LT4,false,GSLC_COL_GRAY,
    0,10,GSLC_COL_GRAY);
  PumpSTARTslide = pElemRef;

  // Create seekbar E_Heat_Seek
  pElemRef = gslc_ElemXSeekbarCreate(&m_gui,E_Heat_Seek,SetPoints,&m_sXSeekbar6,
    (gslc_tsRect){126,85,160,20},70,110,85,
    4,2,8,GSLC_COL_BLUE,GSLC_COL_GRAY,GSLC_COL_BLUE_DK2,false);
  gslc_ElemXSeekbarSetPosFunc(&m_gui,pElemRef,&CbSlidePos);
  gslc_ElemXSeekbarSetStyle(&m_gui,pElemRef,true,GSLC_COL_BLUE_LT4,false,GSLC_COL_GRAY,
    0,10,GSLC_COL_GRAY);
  HeaterSTOPslide = pElemRef;

  // Create seekbar E_ELEM_SEEKBAR7
  pElemRef = gslc_ElemXSeekbarCreate(&m_gui,E_ELEM_SEEKBAR7,SetPoints,&m_sXSeekbar7,
    (gslc_tsRect){126,153,160,20},30,60,60,
    4,2,8,GSLC_COL_BLUE,GSLC_COL_GRAY,GSLC_COL_BLUE_DK2,false);
  gslc_ElemXSeekbarSetPosFunc(&m_gui,pElemRef,&CbSlidePos);
  gslc_ElemXSeekbarSetStyle(&m_gui,pElemRef,true,GSLC_COL_BLUE_LT4,false,GSLC_COL_GRAY,
    0,10,GSLC_COL_GRAY);
  PumpSTOPslide = pElemRef;

  // Create PumpON runtime modifiable text
  static char m_sDisplayText16[3] = "60";
  pElemRef = gslc_ElemCreateTxt(&m_gui,PumpON,SetPoints,(gslc_tsRect){242,47,24,16},
    (char*)m_sDisplayText16,3,E_BUILTIN10X16);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_GREEN);
  PumpSTARTval = pElemRef;

  // Create HeatOFF runtime modifiable text
  static char m_sDisplayText17[5] = "85";
  pElemRef = gslc_ElemCreateTxt(&m_gui,HeatOFF,SetPoints,(gslc_tsRect){241,118,34,16},//change 34 back to 24 if limited to 2 digits
    (char*)m_sDisplayText17,4,E_BUILTIN10X16);//  change 4 back to 3 if ^^^
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_GREEN);
  HeaterSTOPval = pElemRef;

  // Create PumpOFF runtime modifiable text
  static char m_sDisplayText18[3] = "70";
  pElemRef = gslc_ElemCreateTxt(&m_gui,PumpOFF,SetPoints,(gslc_tsRect){240,179,24,16},
    (char*)m_sDisplayText18,3,E_BUILTIN10X16);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_GREEN);
  PumpSTOPval = pElemRef;

  // create E_StartBtn button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_StartBtn,SetPoints,
    (gslc_tsRect){8,35,105,49},(char*)"Back",0,E_BUILTIN15X24,&CbBtnCommon);
	gslc_ElemSetRoundEn(&m_gui, pElemRef, true);
	gslc_ElemSetFrameEn(&m_gui, pElemRef,false);
	spBACKbtn = pElemRef;
  // -----------------------------------
  // PAGE: Running
  
  
  // Create E_PumpStat button with modifiable text label
  static char m_strbtn5[9] = "Pump";
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_PumpStat,Running,
    (gslc_tsRect){15,18,80,40},
    (char*)m_strbtn5,9,E_BUILTIN5X8,&CbBtnCommon);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLUE_DK2,GSLC_COL_GREEN,GSLC_COL_BLUE_DK1);
  gslc_ElemSetRoundEn(&m_gui, pElemRef, true);
  gslc_ElemSetFrameEn(&m_gui,pElemRef,false);
  m_PumpStat6 = pElemRef;
  
  // Create toggle button E_Pumptog
  pElemRef = gslc_ElemXTogglebtnCreate(&m_gui,E_Pumptog,Running,&m_asXToggle1,
    (gslc_tsRect){127,27,35,20},GSLC_COL_GRAY,GSLC_COL_GREEN,GSLC_COL_GRAY_DK3,
    true,false,&CbBtnCommon);
  m_pumptog = pElemRef;
  gslc_ElemSetGroup(&m_gui,pElemRef,GSLC_GROUP_ID_pump);
  
  // Create E_Heatstat button with modifiable text label
  static char m_strbtn6[9] = "Heater";
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_Heatstat,Running,
    (gslc_tsRect){14,72,80,40},
    (char*)m_strbtn6,9,E_BUILTIN5X8,&CbBtnCommon);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_RED,GSLC_COL_RED,GSLC_COL_BLUE_DK1);
  gslc_ElemSetRoundEn(&m_gui, pElemRef, true);
  gslc_ElemSetFrameEn(&m_gui,pElemRef,false);
  m_HeatStat6 = pElemRef;
  
  // Create toggle button E_Heattog
  pElemRef = gslc_ElemXTogglebtnCreate(&m_gui,E_Heattog,Running,&m_asXToggle3,
    (gslc_tsRect){126,79,35,20},GSLC_COL_GRAY,GSLC_COL_RED,GSLC_COL_GRAY_DK3,
    true,false,&CbBtnCommon);
  m_heattog = pElemRef;
  gslc_ElemSetGroup(&m_gui,pElemRef,GSLC_GROUP_ID_heat);
  
  // Create E_ELEM_TEXT19 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT19,Running,(gslc_tsRect){189,29,96,16},
    (char*)"Cur Temp",0,E_BUILTIN10X16);
  
  // Create E_ELEM_TEXTtemp runtime modifiable text
  static char m_sDisplayText20[10] = "0";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXTtemp,Running,(gslc_tsRect){183,55,100,16},
    (char*)m_sDisplayText20,9,E_BUILTIN10X16);
  gslc_ElemSetTxtAlign(&m_gui,pElemRef,GSLC_ALIGN_MID_RIGHT);
  gslc_ElemSetFrameEn(&m_gui,pElemRef,false);
  m_pElemOutTxt20 = pElemRef;
  
  // Create E_ELEM_TEXT21 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT21,Running,(gslc_tsRect){262,45,6,8},
    (char*)"o",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXTrun text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXTrun,Running,(gslc_tsRect){186,114,96,16},
    (char*)"Run Time",0,E_BUILTIN10X16);
  
  // Create E_ELEM_Time runtime modifiable text
  static char m_sDisplayText23[9] = "";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_Time,Running,(gslc_tsRect){185,136,96,16},
    (char*)m_sDisplayText23,9,E_BUILTIN10X16);
  m_pElemOutTime = pElemRef;
  
  // Create E_ELEM_rem text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_rem,Running,(gslc_tsRect){128,175,131,16},
    (char*)"Remaining Est",0,E_BUILTIN10X16);
  
  // Create E_ELEM_ttg runtime modifiable text
  static char m_sDisplayText25[9] = "";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_ttg,Running,(gslc_tsRect){186,199,96,16},
    (char*)m_sDisplayText25,9,E_BUILTIN10X16);
  m_pElem_ttg = pElemRef;
  
  // create E_ELEM_BTN7 button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN7,Running,
    (gslc_tsRect){15,125,82,46},(char*)"Back",0,E_BUILTIN15X24,&CbBtnCommon);
  gslc_ElemSetRoundEn(&m_gui, pElemRef, true);
  gslc_ElemSetFrameEn(&m_gui,pElemRef,false);
  runBACKbtn = pElemRef;

  // create E_ELEM_BTN8 button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN8,Running,(gslc_tsRect){15,185,82,46},(char*)"Stop",0,E_BUILTIN15X24,&CbBtnCommon);
  gslc_ElemSetRoundEn(&m_gui, pElemRef, true);
  gslc_ElemSetFrameEn(&m_gui,pElemRef,false);
runSTOPbtn = pElemRef;
  // -----------------------------------
  // PAGE: Finished
  
  
  // Create E_ELEM_TEXT26 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT26,Finished,(gslc_tsRect){29,153,240,40},
    (char*)"Finished",0,E_BUILTIN25X40);

  // Create progress bar E_ELEM_PROGRESS2 
  pElemRef = gslc_ElemXProgressCreate(&m_gui,E_ELEM_PROGRESS2,Finished,&m_sXBarGauge2,
    (gslc_tsRect){29,60,240,20},0,100,100,GSLC_COL_GREEN,false);
  m_pElemProgress2 = pElemRef;
//<InitGUI !End!>

//<Startup !Start!>
  gslc_GuiRotate(&m_gui, 1);
//<Startup !End!>

}

#endif // end _GUISLICE_GEN_H
