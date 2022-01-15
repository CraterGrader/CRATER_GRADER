# Foreword

I have forked this from [Tpope's Vividchalk](https://github.com/tpope/vim-vividchalk)
and slightly modified it.

All credit goes to him. Check him out. He has many great Vim plugins.

## Installation

```vimL
call plug#begin()
  Plug 'ParamagicDev/vim-medic_chalk'
call plug#end()

colorscheme medic_chalk
```

![Gif of colorscheme](https://thumbs.gfycat.com/MistyScientificBlackmamba-size_restricted.gif)

Below is a sample for `~/.Xresources` taken from RomainL

    *.foreground: #FFFFFF
    *.background: #000000
    *.color0:     #000000
    *.color8:     #111111
    *.color1:     #FFA500
    *.color9:     #FF6600
    *.color2:     #66FF00
    *.color10:    #99CC99
    *.color3:     #FFCC00
    *.color11:    #FFEE98
    *.color4:     #5F87AF
    *.color12:    #33C9C9
    *.color5:     #9933CC
    *.color13:    #AA1BF2
    *.color6:     #88B5C3
    *.color14:    #AABBEE
    *.color7:     #FFFFFF
    *.color15:    #808080

## Differences from VividChalk

There aren't many differences,

I changed the way the auto completion menu is handled.

I also changed the fact that the current cursor row was underlined.

I also changed how splits appears, colorcolumn appears, and
how matched parentheses appear.

Altered fallback values for GUI StatusLines and Normal highlighting.

Altered the way non-current status line is highlighted.

Added a highlight link for NERDTree (Sorry TPope, I know you like NETRW w/ vinegar)

Added a color palette to the README for easier porting of the colorscheme.

Overall, it's pretty much the same just with a few adjustments. TPope did
All the heavy lifting on this one.
