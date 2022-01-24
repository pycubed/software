'''
from debugcolor import co
print(co(msg='hello!',color='red'))
'''
_h="\033["
_e='\033[0;39;49m'

_c={
'red'    :'1',
'green'  :'2',
'orange' :'3',
'blue'   :'4',
'pink'   :'5',
'teal'   :'6',
'white'  :'7',
'gray'   :'9'}

_f={
'normal' :'0',
'bold'   :'1',
'ulined' :'4'}

def co(msg,color='gray',fmt='normal'):
    return _h+_f[fmt]+';3'+_c[color]+'m'+msg+_e
