#ifndef INC_MAZE_HPP_
#define INC_MAZE_HPP_

#include "main.h"



class Maze{
    private:
        class Node{
        public:
            class Wall{
                public:
                    uint8_t up = 0;
                    uint8_t down = 0;
                    uint8_t left = 0;
                    uint8_t right = 0;
            };
            Wall wall;
            uint8_t cost = 255;
        };
        class CoordI{
            public:
                uint8_t x;
                uint8_t y;
            
                // →0, ↑1, ←2, ↓3
                int8_t muki;
        };
    public:
        
        Node node[16][16];
        CoordI coord;

        Maze(){
            for (uint8_t i = 0; i < 16; i++){
                for (uint8_t j = 0; j < 16; j++){
                    if (i == 0) node[i][j].wall.left = 1;
                    if (i == 15) node[i][j].wall.right = 1;
                    if (j == 0) node[i][j].wall.down = 1;
                    if (j == 15) node[i][j].wall.up = 1;
                }
            }
        }
        void wall_update(CoordI c, uint8_t up, uint8_t down, uint8_t left, uint8_t right);
        bool is_wall_up(void);
        bool is_wall_left(void);
        bool is_wall_right(void);
        bool is_wall_left2(void);
        bool is_wall_right2(void);
        bool is_wall_down2(void);
};      

#endif