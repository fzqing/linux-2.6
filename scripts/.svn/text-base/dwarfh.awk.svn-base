BEGIN {
	print "#ifndef  _ELF_DWARF_H"
		print "/* Machine generated from dwarf2.h by scripts/dwarfh.awk */"	
}
$2 == "=" {
	gsub(/,/, "", $3)
	print "#define " $1 "\t " $3
}
$1 == "#define" {
	print $0
	while( index($0,"\\") == length($0)){
		getline
		print $0
	}
}
/.*/ {}
END {
	print "#endif"
}
