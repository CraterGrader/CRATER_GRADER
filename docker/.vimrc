" Plug-ins go between these call functions
call plug#begin('~/.vim/plugged')

" Color scheme
Plug 'https://github.com/flazz/vim-colorschemes.git' " Lots of options
Plug 'https://github.com/vim-scripts/CSApprox.git' " Convert for terminal
Plug 'https://github.com/ParamagicDev/vim-medic_chalk.git' " Good colorscheme
call plug#end()

" Color scheme (from plug-in)
set t_Co=256
colorscheme medic_chalk

" Search highlighting
set hlsearch
hi Search ctermbg=cyan
hi Search ctermfg=red

" Set tabs to be 2 spaces
set expandtab " Insert spaces instead of tabs
set smarttab " forces use of shiftwidth and tabstop
set shiftwidth=2 " Change the number of spaces inserted for indentation
set tabstop=2 " Tabs insert 2 spaces

" Display line numbers
set number

" Format text past 80 columns
"set textwidth=80
"set colorcolumn=+1
"let w:m1=matchadd('ErrorMsg', '\%>80v.\+', -1)

"Highlight active line
"Enables cursor line position tracking:
set cursorline
" Removes the underline causes by enabling cursorline:
highlight clear CursorLine
" Sets the line numbering to red background:
highlight CursorLineNR ctermbg=white

" Automatically include closing braces
inoremap { {}<ESC>ha
inoremap ( ()<ESC>ha
inoremap [ []<ESC>ha
inoremap " ""<ESC>ha
inoremap ' ''<ESC>ha

" Ignore presets
let g:python_recommended_style = 0

" Highlight search 
set hlsearch
" Press Space to turn off highlighting and clear any message already displayed.
nnoremap <silent> <Space> :nohlsearch<Bar>:echo<CR>"
