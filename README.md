# MicroNaos_open
マイクロマウスというロボット競技の、クラシックマウス部門用のロボットです。

<img src=https://user-images.githubusercontent.com/48354170/200001577-363b2204-1d9d-4d47-936f-57a51cea68ca.jpg width=500px>

## ディレクトリ
### DXF

基板外形のDXFファイルです

### firmware
STM32向けのソースコードです。主にC++で記述しています。STM32HAL、CMSIS、SEGGER-RTTなどのライブラリも含まれています。

### PCB
機体の大部分を構成しているプリント基板及び、その回路のファイルです。Kicadを用いて作成しました
公開するにあたってライセンスの問題がないように編集を行ったため、回路シンボルが基板発注時と少し異なっています。
