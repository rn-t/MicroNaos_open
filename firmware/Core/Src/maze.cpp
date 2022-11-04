#include "main.h"
#include "maze.hpp"
#include "arm_math.h"
#include "SEGGER_RTT.h"


void Maze::wall_update(CoordI c, uint8_t up, uint8_t down, uint8_t left, uint8_t right){
    uint8_t temp;
    if(c.muki == 0){
        temp = up;
        up = left;
        left = down;
        down = right;
        right = temp;
    }else if (c.muki == 2){
        temp = up;
        up = right;
        right = down;
        down = left;
        left = temp;
    }else if (c.muki == 3){
        temp = up;
        up = down;
        down = temp;

        temp = left;
        left = right;
        right = temp;
    }

    if (node[c.x][c.y].wall.up == 0){
        node[c.x][c.y].wall.up = up;
    }
    if (node[c.x][c.y].wall.down == 0){
        node[c.x][c.y].wall.down = down;
    }
    if (node[c.x][c.y].wall.left == 0){
        node[c.x][c.y].wall.left = left;
    }
    if (node[c.x][c.y].wall.right == 0){
        node[c.x][c.y].wall.right = right;
    }

    if (c.y > 0){
        //y座標が0でないとき、1つ下側のノード情報も更新する。
        if(node[c.x][c.y + 1].wall.down == 0){
            node[c.x][c.y + 1].wall.down = up;
        }
    }
    if (c.y < 15){
        //y座標が15でないとき、1つ上側のノード情報も更新する。
        if(node[c.x][c.y - 1].wall.up == 0){
            node[c.x][c.y - 1].wall.up = down;
        } 
    }

    if (c.x > 0){
        //x座標が0でないとき、1つ右側のノード情報も更新する。
        if(node[c.x - 1][c.y].wall.right == 0){
            node[c.x - 1][c.y].wall.right = left;
        }
        
    }
    if (c.x < 15){
        //x座標が0でないとき、1つ右側のノード情報も更新する。
       if(node[c.x + 1][c.y].wall.left == 0){
            node[c.x + 1][c.y].wall.left = right;
        } 
    }

}

bool Maze::is_wall_up(void){
    if (coord.muki == 0 && node[coord.x][coord.y].wall.right > 0) return true;
    if (coord.muki == 1 && node[coord.x][coord.y].wall.up > 0) return true;
    if (coord.muki == 2 && node[coord.x][coord.y].wall.left > 0) return true;
    if (coord.muki == 3 && node[coord.x][coord.y].wall.down > 0) return true;
    return false;
}
bool Maze::is_wall_left(void){
    if (coord.muki == 0 && node[coord.x][coord.y].wall.up > 0) return true;
    if (coord.muki == 1 && node[coord.x][coord.y].wall.left > 0) return true;
    if (coord.muki == 2 && node[coord.x][coord.y].wall.down > 0) return true;
    if (coord.muki == 3 && node[coord.x][coord.y].wall.right > 0) return true;
    return false;
}
bool Maze::is_wall_right(void){
    if (coord.muki == 0 && node[coord.x][coord.y].wall.down > 0) return true;
    if (coord.muki == 1 && node[coord.x][coord.y].wall.right > 0) return true;
    if (coord.muki == 2 && node[coord.x][coord.y].wall.up > 0) return true;
    if (coord.muki == 3 && node[coord.x][coord.y].wall.left > 0) return true;
    return false;
}

bool Maze::is_wall_left2(void){
    if (coord.muki == 0 && node[coord.x][coord.y].wall.up == 1) return true;
    if (coord.muki == 1 && node[coord.x][coord.y].wall.left == 1) return true;
    if (coord.muki == 2 && node[coord.x][coord.y].wall.down == 1) return true;
    if (coord.muki == 3 && node[coord.x][coord.y].wall.right == 1) return true;
    return false;
}
bool Maze::is_wall_right2(void){
    if (coord.muki == 0 && node[coord.x][coord.y].wall.down == 1) return true;
    if (coord.muki == 1 && node[coord.x][coord.y].wall.right == 1) return true;
    if (coord.muki == 2 && node[coord.x][coord.y].wall.up == 1) return true;
    if (coord.muki == 3 && node[coord.x][coord.y].wall.left == 1) return true;
    return false;
}
bool Maze::is_wall_down2(void){
    if (coord.muki == 0 && node[coord.x][coord.y].wall.left == 1) return true;
    if (coord.muki == 1 && node[coord.x][coord.y].wall.down == 1) return true;
    if (coord.muki == 2 && node[coord.x][coord.y].wall.right == 1) return true;
    if (coord.muki == 3 && node[coord.x][coord.y].wall.up == 1) return true;
    return false;
}