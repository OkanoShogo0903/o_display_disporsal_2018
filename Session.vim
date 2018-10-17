let SessionLoad = 1
let s:so_save = &so | let s:siso_save = &siso | set so=0 siso=0
let v:this_session=expand("<sfile>:p")
silent only
cd ~/catkin_ws/src/open3d_workspace
if expand('%') == '' && !&modified && line('$') <= 1 && getline(1) == ''
  let s:wipebuf = bufnr('%')
endif
set shortmess=aoO
badd +1 term://.//3828:/bin/bash
badd +1 term://.//2995:/bin/bash
badd +1 term://.//3040:/bin/bash
badd +17 term://.//3932:/bin/bash
badd +1 README.md\[
badd +7 README.md
badd +1 term://.//4786:/bin/bash
badd +114 scripts/main.py
badd +11 term://.//23671:/bin/bash
badd +16 scripts/interactive_visualization.py
badd +4 term://.//5523:/bin/bash
badd +153 ~/catkin_ws/src/realsense2_camera/launch/rs_rgbd.launch
badd +35 ~/catkin_ws/src/realsense2_camera/cfg/origin_rs435_params.cfg
badd +1 scripts/reconfigure.sh
badd +177 reference.md
badd +1 hoge
badd +11 scripts/print_help.py
badd +1 term://.//4888:/bin/bash
badd +34 samples/scripts/removal.py
badd +77 term://.//15647:/bin/bash
badd +97 scripts/util.py
badd +2 samples/scripts/pointcloud.py
badd +13 samples/TestData/Crop/cropped.json
badd +1 samples/TestData/Crop/fragment.ply
badd +39 term://.//3746:/bin/bash
badd +50 samples/scripts/color_optimization.py
badd +3 samples/scripts/trajectory_io.py
badd +353 term://.//4090:/bin/bash
badd +28 term://.//3736:/bin/bash
badd +32 ~/numpy_train.py
badd +1 term://.//16700:/bin/bash
badd +11 term://.//4064:/bin/bash
badd +67 ~/catkin_ws/src/o_gpsr_2018/scripts/google_tts.py
badd +1 ~/catkin_ws/src/o_display_disporsal_2018/scripts/main.py
badd +245 term://.//12929:/bin/bash
badd +1 launch/open3d_workspace.launch
badd +7 ~/catkin_ws/src/o_display_disporsal_2018/launch/master.launch
badd +1 package.xml
badd +8 term://.//18953:/bin/bash
badd +19 term://.//26060:/bin/bash
badd +1 term://.//3017:/bin/bash
badd +1 ~/labpass.txt
badd +12 ~/.ros/log/da6f88ac-cd23-11e8-952f-7c7a91806f92/open3d_workspace-2.log
badd +0 term://.//6556:/bin/bash
argglobal
silent! argdel *
set splitbelow splitright
wincmd _ | wincmd |
split
1wincmd k
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd w
wincmd w
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winminheight=1 winminwidth=1 winheight=1 winwidth=1
exe '1resize ' . ((&lines * 20 + 21) / 43)
exe 'vert 1resize ' . ((&columns * 74 + 75) / 150)
exe '2resize ' . ((&lines * 20 + 21) / 43)
exe 'vert 2resize ' . ((&columns * 75 + 75) / 150)
exe '3resize ' . ((&lines * 19 + 21) / 43)
exe 'vert 3resize ' . ((&columns * 75 + 75) / 150)
exe '4resize ' . ((&lines * 19 + 21) / 43)
exe 'vert 4resize ' . ((&columns * 74 + 75) / 150)
argglobal
if bufexists('term://.//2995:/bin/bash') | buffer term://.//2995:/bin/bash | else | edit term://.//2995:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 177 - ((19 * winheight(0) + 10) / 20)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
177
normal! 0
wincmd w
argglobal
if bufexists('term://.//3017:/bin/bash') | buffer term://.//3017:/bin/bash | else | edit term://.//3017:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 32 - ((19 * winheight(0) + 10) / 20)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
32
normal! 040|
wincmd w
argglobal
if bufexists('term://.//3040:/bin/bash') | buffer term://.//3040:/bin/bash | else | edit term://.//3040:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 19 - ((18 * winheight(0) + 9) / 19)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
19
normal! 0
wincmd w
argglobal
if bufexists('term://.//6556:/bin/bash') | buffer term://.//6556:/bin/bash | else | edit term://.//6556:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 1 - ((0 * winheight(0) + 9) / 19)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
wincmd w
exe '1resize ' . ((&lines * 20 + 21) / 43)
exe 'vert 1resize ' . ((&columns * 74 + 75) / 150)
exe '2resize ' . ((&lines * 20 + 21) / 43)
exe 'vert 2resize ' . ((&columns * 75 + 75) / 150)
exe '3resize ' . ((&lines * 19 + 21) / 43)
exe 'vert 3resize ' . ((&columns * 75 + 75) / 150)
exe '4resize ' . ((&lines * 19 + 21) / 43)
exe 'vert 4resize ' . ((&columns * 74 + 75) / 150)
tabedit README.md
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winminheight=1 winminwidth=1 winheight=1 winwidth=1
exe 'vert 1resize ' . ((&columns * 74 + 75) / 150)
exe 'vert 2resize ' . ((&columns * 75 + 75) / 150)
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 9 - ((5 * winheight(0) + 20) / 40)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
9
normal! 018|
wincmd w
argglobal
if bufexists('~/catkin_ws/src/o_display_disporsal_2018/scripts/main.py') | buffer ~/catkin_ws/src/o_display_disporsal_2018/scripts/main.py | else | edit ~/catkin_ws/src/o_display_disporsal_2018/scripts/main.py | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 121 - ((19 * winheight(0) + 20) / 40)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
121
normal! 029|
wincmd w
exe 'vert 1resize ' . ((&columns * 74 + 75) / 150)
exe 'vert 2resize ' . ((&columns * 75 + 75) / 150)
tabedit launch/open3d_workspace.launch
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winminheight=1 winminwidth=1 winheight=1 winwidth=1
exe 'vert 1resize ' . ((&columns * 75 + 75) / 150)
exe 'vert 2resize ' . ((&columns * 74 + 75) / 150)
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 2 - ((1 * winheight(0) + 20) / 40)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
2
normal! 072|
lcd ~/catkin_ws/src/open3d_workspace
wincmd w
argglobal
if bufexists('~/catkin_ws/src/o_display_disporsal_2018/launch/master.launch') | buffer ~/catkin_ws/src/o_display_disporsal_2018/launch/master.launch | else | edit ~/catkin_ws/src/o_display_disporsal_2018/launch/master.launch | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 1 - ((0 * winheight(0) + 20) / 40)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
lcd ~/catkin_ws/src/o_display_disporsal_2018
wincmd w
2wincmd w
exe 'vert 1resize ' . ((&columns * 75 + 75) / 150)
exe 'vert 2resize ' . ((&columns * 74 + 75) / 150)
tabnext 3
if exists('s:wipebuf') && getbufvar(s:wipebuf, '&buftype') isnot# 'terminal'
  silent exe 'bwipe ' . s:wipebuf
endif
unlet! s:wipebuf
set winheight=1 winwidth=20 winminheight=1 winminwidth=1 shortmess=filnxtToO
let s:sx = expand("<sfile>:p:r")."x.vim"
if file_readable(s:sx)
  exe "source " . fnameescape(s:sx)
endif
let &so = s:so_save | let &siso = s:siso_save
doautoall SessionLoadPost
unlet SessionLoad
" vim: set ft=vim :
