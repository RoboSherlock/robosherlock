define printuchar
 dont-repeat
 set $i=0
 set $s=$arg0
 set $n=$arg1
 while $i<$n
  if 0 == *($s + $i)
    loop_break
  end
  printf "%c", *($s + $i++)
 end
 printf "\n"
end

define printusref
 dont-repeat
 set $i=0
 set $s=$arg0->iv_pUChars
 set $n=$arg0->iv_uiLength
 while $i<$n
  if 0 == *($s + $i)
    loop_break
  end
  printf "%c", *($s + $i++)
 end
 printf "\n"
end


define printus
 dont-repeat
 set $i=0
 set $s=$arg0->fArray
 set $n=$arg0->fLength
 while $i<$n
  if 0 == *($s + $i)
    loop_break
  end
  printf "%c", *($s + $i++)
 end
 printf "\n"
end
