// Menus.cpp

/*
 * Copyright (C) 2014, by Author: Eldon R. Brown (ERB) - WA0UWH
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <Arduino.h>
#include "A1Main.h"
#include "Menus.h"
#include "NonVol.h"
#include "PotKnob.h"
#include "Encoder01.h"
#include "ButtonUtil.h"
#include "MorseCode.h"
#include "debug.h"

byte menuActive = 0;
byte menuPrev = 0;
unsigned long menuIdleTimer = 0;
boolean menuCycle = true;

// Local Functions
void checkKnob(int menu);
void checkButtonMenu();
void updateDisplayMenu(int menu);

// ###############################################################################
void doMenus(int menu) {
#define DEBUG(x...)
//#define DEBUG(x...) debugUnique(x)    // UnComment for Debug

    DEBUG(P("%s/%d: Start Menu"), __func__, __LINE__);
    
    if (menuIdleTimeOut < 5 * SECs) menuIdleTimeOut = 0;

    if (!menuIdleTimer) menuIdleTimer = millis() + menuIdleTimeOut;

    DEBUG(P("%s/%d: menuIdleTimeOut= %ld"), __func__, __LINE__, menuIdleTimeOut);
    
    checkKnob(menu);
    checkButtonMenu();

    if (menuIdleTimeOut && menuIdleTimer && menuIdleTimer < millis()) { // If IdleTimeOut, Abort
      DEBUG(P("%s/%d: Stop Menu"), __func__, __LINE__);
      menuCycle = true;
      menuIdleTimer = 0;
      menuActive = 0;
      refreshDisplay++;
    }

    if (menuActive) updateDisplayMenu(menu);
}


// ###############################################################################
void checkKnob(int menu) {
#define DEBUG(x...)
//#define DEBUG(x...) debugUnique(x)    // UnComment for Debug
    int dir;
    
    DEBUG(P("%s/%d:"), __func__, __LINE__);
    
    dir = 0;
      
    #ifdef USE_POT_KNOB
        dir = getPotDir(); // Get Tuning Direction from POT Knob
    #endif // USE_POT_KNOB
      
    #ifdef USE_ENCODER
        dir = getEncoderDir(); // Get Tuning Direction from Encoder Knob
    #endif // USE_ENCODER

    
    if (!dir) return;
 
    menuIdleTimer = 0;
    
    if (menuCycle) { // Cycle or Page Through Menus    
        dir = (dir > 0) - (dir < 0); // Remove Magnitude, save +/- Direciton
        menuActive = constrain (menuActive + dir, 1, MENUS-1);
        refreshDisplay++;
        updateDisplayMenu(menuActive);
        return;
    }
      
    DEBUG(P("%s/%d: Menu = %d"), __func__, __LINE__, menu);
    
    switch(menu) {
        #ifdef USE_OPERATE_60M
            case M_OPT60M:
              operate60m = !operate60m;
              break;
        #endif // USE_OPERATE_60M
        case M_CW_WPM:
          cw_wpm += dir;        
          cw_wpm = constrain (cw_wpm, 1, 99);
          break;
        case M_QRSS_DIT_TIME:
          if (qrssDitTime > SECs) {
              qrssDitTime = (qrssDitTime + dir * SECs) / SECs * SECs;  // Truncate to Seconds
              qrssDitTime = max(qrssDitTime, 1 * SECs);
          }
          else qrssDitTime += dir * 10;       
          qrssDitTime = constrain (qrssDitTime, 250, 60 * SECs); //MSECs
          break;
          
        case M_PARK_TIMEOUT:
          if (parkTimeOut > MINs) {
              parkTimeOut = (parkTimeOut + dir * MINs) / MINs * MINs;  // Truncate to Minutes
              parkTimeOut = max(parkTimeOut, 1 * MINs);
          }
          else  parkTimeOut += dir * 5 * SECs;
          parkTimeOut = parkTimeOut / SECs * SECs;
          parkTimeOut = constrain (parkTimeOut, 0, 240 * MINs); //MSECs
          break;
          
        #ifdef USE_BLINK_DIGIT_MODE 
            case M_BLINK_PERIOD:
              if (blinkPeriod > SECs) {
                  blinkPeriod = (blinkPeriod + dir * SECs) / SECs * SECs;  // Truncate to Seconds
                  blinkPeriod = max(blinkPeriod, 10 * SECs);
              }
              else  blinkPeriod += dir * 10;
              blinkPeriod = constrain (blinkPeriod, 100, 10 * SECs); //MSECs
              break;
            case M_BLINK_RATIO:
              blinkRatio += dir * 5;
              blinkRatio = constrain (blinkRatio, 20, 95); // Percent 
              break;
        #endif // USE_BLINK_DIGIT_MODE
          
        case M_TIMEOUT:
          if (menuIdleTimeOut > MINs) {
              menuIdleTimeOut = (menuIdleTimeOut + dir * MINs) / MINs * MINs;  // Truncate to Minutes
              menuIdleTimeOut = max(menuIdleTimeOut, 1 * MINs);
          }
          else  menuIdleTimeOut += dir * 5 * SECs;
          if (menuIdleTimeOut > 3 * 10 * MINs) menuIdleTimeOut = 0;
          menuIdleTimeOut = menuIdleTimeOut / SECs * SECs; // Truncate to Seconds
          menuIdleTimeOut = constrain (menuIdleTimeOut, 0, 10 * MINs); //MSECs
          break;

        default:;
    }
    refreshDisplay++;
}


// ###############################################################################
void updateDisplayMenu(int menu) {
#define DEBUG(x...)
//#define DEBUG(x...) debugUnique(x)    // UnComment for Debug

    DEBUG(P("%s/%d:"), __func__, __LINE__);

  if (refreshDisplay > 0) {
      refreshDisplay--;

      //sprintf(c, P("%0.2d-Menu:"), menu);
      //printLineCEL(MENU_PROMPT_LINE, c);      
      DEBUG(P("%s/%d: Menu = %d"), __func__, __LINE__, menu);
      switch (menu) {
          case 0: // Exit Menu System
             sprintf(c, P("Exit Menu"), menu);
             printLineCEL(MENU_PROMPT_LINE, c);
             printLineCEL(MENU_ITEM_LINE, P(" "));
             break;
             
          #ifdef USE_OPERATE_60M
              case M_OPT60M:
                 sprintf(c, P("%0.2dOPTION 60M"), menu);
                 printLineCEL(MENU_PROMPT_LINE, c);
                 sprintf(c, P("INCLUDE: %s"), operate60m ? P2("YES") : P2("NO"));
                 if (!menuCycle) sprintf(c, P2("%s<"), c);
                 printLineCEL(MENU_ITEM_LINE, c);
              break;
          #endif // USE_OPERATE_60M
          
          case M_CW_WPM:
             sprintf(c, P("%0.2dMACRO CW SPD"), menu);
             printLineCEL(MENU_PROMPT_LINE, c);
             sprintf(c, P("WPM: %0.2d"), cw_wpm);
             if (!menuCycle) sprintf(c, P2("%s<"), c);
             printLineCEL(MENU_ITEM_LINE, c);
             break;
          case M_QRSS_DIT_TIME:
             sprintf(c, P("%0.2dMACRO QRSS DIT"), menu);
             printLineCEL(MENU_PROMPT_LINE, c);
             if (qrssDitTime > SECs) sprintf(c, P(" SECs: %0.2d"), qrssDitTime / SECs);
             else sprintf(c, P("MSECs: %d"), qrssDitTime);
             if (!menuCycle) sprintf(c, P2("%s<"), c);
             printLineCEL(MENU_ITEM_LINE, c);
             break;

          case M_PARK_TIMEOUT:
             sprintf(c, P("%0.2dPark TimeOut"), menu);
             printLineCEL(MENU_PROMPT_LINE, c);
             if (parkTimeOut > MINs) sprintf(c, P("MINs: %0.2d"), parkTimeOut / MINs);
             else sprintf(c, P("SECs: %d"), parkTimeOut / SECs);
             if (!menuCycle) sprintf(c, P2("%s<"), c);
             if (!parkTimeOut) sprintf(c, P2("%s - OFF"), c);
             printLineCEL(MENU_ITEM_LINE, c);
             break;
             
          #ifdef USE_BLINK_DIGIT_MODE
              case M_BLINK_PERIOD:
                 sprintf(c, P("%0.2dBlink Period"), menu);
                 printLineCEL(MENU_PROMPT_LINE, c);
                 if (blinkPeriod > SECs) sprintf(c, P("SECs: %0.2d"), blinkPeriod / SECs);
                 else sprintf(c, P("MSECs: %d"), blinkPeriod);
                 if (!menuCycle) sprintf(c, P2("%s<"), c);
                 printLineCEL(MENU_ITEM_LINE, c);
                 break;
              case M_BLINK_RATIO:
                 sprintf(c, P("%0.2dBlink"), menu);
                 printLineCEL(MENU_PROMPT_LINE, c);
                 sprintf(c, P("Ratio: %d%%"), blinkRatio);
                 if (!menuCycle) sprintf(c, P2("%s<"), c);
                 printLineCEL(MENU_ITEM_LINE, c);
                 break;
          #endif // USE_BLINK_DIGIT_MODE 
          
          case M_TIMEOUT:
             sprintf(c, P("%0.2dMenu TimeOut"), menu);
             printLineCEL(MENU_PROMPT_LINE, c);
             if (menuIdleTimeOut > MINs) sprintf(c, P("MINs: %0.2d"), menuIdleTimeOut / MINs);
             else sprintf(c, P("SECs: %d%"), menuIdleTimeOut / SECs);
             if (!menuCycle) sprintf(c, P2("%s<"), c);
             if (!menuIdleTimeOut) sprintf(c, P2("%s - OFF"), c);
             printLineCEL(MENU_ITEM_LINE, c);
             break;

          default: // A Blank Menu Item
             sprintf(c, P("%0.2dMenu:"), menu);
             printLineCEL(MENU_PROMPT_LINE, c);
             printLineCEL(MENU_ITEM_LINE, P(" -"));
             break;
      }
  }
}

// ###############################################################################
void checkButtonMenu() {
#define DEBUG(x ...)
//#define DEBUG(x ...) debugUnique(x)    // UnComment for Debug
//#define DEBUG(x ...) debug(x)    // UnComment for Debug
  int btn;
  static int btnPrev;
  
  btn = btnDown();
  if (btn) DEBUG(P("%s/%d: btn %d"), __func__, __LINE__, btn);

  menuPrev = menuActive;
  switch (btn) {
    case 0: btnPrev = btn; return; // Abort
    case UP_BTN: menuCycle = true; menuActive = constrain (menuActive+1, 1, MENUS-1); break;
    case DN_BTN: menuCycle = true; menuActive = constrain (menuActive-1, 1, MENUS-1); break;
    case LT_BTN:
         switch (getButtonPushMode(btn)) { 
             #ifdef USE_EEPROM
                 case DOUBLE_PRESS: eePromIO(EEP_LOAD); break;
                 case LONG_PRESS:   eePromIO(EEP_SAVE); break;
             #endif // USE_EEPROM
             default: break; // Do nothing
         }
         break;
    case RT_BTN:
        switch (getButtonPushMode(btn)) {
            case MOMENTARY_PRESS: menuCycle = !menuCycle; break;
            case DOUBLE_PRESS:    menuCycle = true; menuActive = 0; refreshDisplay+=2; break; // Return to VFO Display Mode
            default: break; // Do nothing
        } 
        break;
        #ifdef USE_ENCODER01
            case ENC_KNOB: if (btnPrev != btn) readEncoder(btn); break;
        #endif // USE_ENCODER01
     default: decodeAux(btn); break;
  }
  btnPrev = btn;
  DEBUG(P("%s/%d: MenuActive %d"), __func__, __LINE__, menuActive);
  menuIdleTimer = 0;
  refreshDisplay++;
  updateDisplayMenu(menuActive);
  deDounceBtnRelease(); // Wait for Button Release
}


// ENd
