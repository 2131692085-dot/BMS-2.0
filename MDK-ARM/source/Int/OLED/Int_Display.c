#include "Int_Display.h"
#include "Int_OLED.h"
#include <stdio.h>
#include "Int_BQ76952.h"

/**
 * 屏幕显示 - 渲染标题 - BMS项目
 */
void __Int_Display_Render_Title(void) {
    Inf_OLED_ShowString(8, 8, "BMS", 16, 1);
    Inf_OLED_ShowChinese(8 + 24 + 16, 8, 2, 16, 1); // "项"
    Inf_OLED_ShowChinese(8 + 24 + 32, 8, 3, 16, 1); // "目"

    Inf_OLED_Refresh();
}

/**
 * 屏幕显示 - 渲染页面
 */
void __Int_Display_Render_Page(uint8_t init) {

    uint8_t percent = 0;
    BQ76952_StatusDef status = Int_BQ76952_OP_Get_SOC(init, &percent);
    if ( status == BQ76952_Status_OK ) {
        uint8_t data[16] = {0};
        sprintf((char *)data, "SOC:%d%%", percent);
        Inf_OLED_ShowString( 8, 8 + 16, data, 16, 1);
        Inf_OLED_Refresh();
    }
}

/**
 * 屏幕显示 - 初始化
 */
void Int_Display_Init(void) {
    Inf_OLED_Init();
    Inf_OLED_Clear();
}

/**
 * 屏幕显示 - 渲染
 */
void Int_Display_Render(uint8_t init) {
    __Int_Display_Render_Title();
    __Int_Display_Render_Page(init);
}
