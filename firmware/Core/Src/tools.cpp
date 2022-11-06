#include "tools.hpp"
/**
 * @brief マウスから見た壁と向きを入力することで、迷路情報に準じた(y軸正方向を向いていると仮定した)壁の情報を返す。
*/
uint8_t tools::get_rotated_wall(uint8_t direction, uint8_t wall){
    uint8_t out = 0;
    switch (direction){
        case Direction::up:
            out = wall;
            break;
        case Direction::left:
            if ((wall & Direction::up) == Direction::up){
                out += Direction::left;
            }
            if ((wall & Direction::left) == Direction::left){
                out += Direction::down;
            }
            if ((wall & Direction::down) == Direction::down){
                out += Direction::right;
            }
            if ((wall & Direction::right) == Direction::right){
                out += Direction::up;
            }
            break;
        case Direction::down:
            if ((wall & Direction::up) == Direction::up){
                out += Direction::down;
            }
            if ((wall & Direction::left) == Direction::left){
                out += Direction::right;
            }
            if ((wall & Direction::down) == Direction::down){
                out += Direction::up;
            }
            if ((wall & Direction::right) == Direction::right){
                out += Direction::left;
            }
            break;
        case Direction::right:
            if ((wall & Direction::up) == Direction::up){
                out += Direction::right;
            }
            if ((wall & Direction::left) == Direction::left){
                out += Direction::up;
            }
            if ((wall & Direction::down) == Direction::down){
                out += Direction::left;
            }
            if ((wall & Direction::right) == Direction::right){
                out += Direction::down;
            }
            break;
    }
    return out;
}


