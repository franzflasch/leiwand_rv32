# file: step_mult.gdb

define step_mult
    set pagination off

    if $argc == 0
        printf "Please specify end position!!!!!!!!!!!!!!!!\n"
        quit
    else
        set $end_pos = $arg0
    end

    printf "end pos %d\n", $end_pos

    while ($pc < $end_pos)
        stepi
    end

    # One more for the nops at the end
    stepi

    quit
end