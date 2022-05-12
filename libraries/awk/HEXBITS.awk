#!/opt/local/bin/gawk -f
{
   bits = strtonum($1); # for conversion if hex
   bitsh = sprintf("0x%X",bits);
   print bits " " bitsh " b"bits2str(bits);
}

function bits2str(bits,        data, mask)
{
    if (bits == 0)
        return "0"

    mask = 1
    for (; bits != 0; bits = rshift(bits, 1))
        data = (and(bits, mask) ? "1" : "0") data

    while ((length(data) % 8) != 0)
        data = "0" data

    return data
}
