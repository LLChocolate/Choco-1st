#include "FIRE_key_event.h"
void deal_key_event()
{
    KEY_MSG_t keymsg;

    while(get_key_msg(& keymsg))     //获得按键就进行处理
    {
        if(keymsg.status == KEY_DOWN)
        {
            switch(keymsg.key)
            {
            case KEY_U:
              //添加按键函数
                break;

             case KEY_D:
                  //添加按键函数
                break;

            case KEY_L:
             //添加按键函数
                break;

            case KEY_R:
               //添加按键函数 
                break;

            case KEY_A:
                 //添加按键函数
                break;



            default:
                break;
            }
        }
        else if(keymsg.status == KEY_HOLD)
        {
            switch(keymsg.key)
            {
            case KEY_U:
              //添加按键长按函数
                break;

            case KEY_D:
               //添加按键长按函数
                break;

            case KEY_L:
               //添加按键长按函数
                break;

            case KEY_R:
               //添加按键长按函数
                break;

            case KEY_A:               
                //添加按键长按函数
                break;

            default:            //不需要处理 cancel
                break;
            }
        }
        else
        {
            
        }
    }
}





