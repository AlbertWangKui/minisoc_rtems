#/bin/bash
echo -e "/* `date` */\n"

echo "#include \"drv_symbols.h\""

echo ""

awk '{ if ($2 == "T") {print "extern int " $3 "();"} \
	   else if ($2 == "B" || $2 == "C" || $2 == "D") {print "extern int " $3 ";"} }' $1

echo -e "\n"

echo "struct drv_shell_cmd g_drv_cmd_tbl[] ="
echo "{"

awk '{ if ($2 == "T") {print "\t{\""  $3  "\", (char *)"  $3  ", SYM_TEXT"  "},"} \
	   else if ($2 == "B") {print "\t{\""  $3  "\", (char *)&"  $3  ", SYM_BSS"  "},"} \
	   else if ($2 == "C") {print "\t{\""  $3  "\", (char *)&"  $3  ", SYM_COMM"  "},"} \
	   else if ($2 == "D") {print "\t{\""  $3  "\", (char *)&"  $3  ", SYM_DATA"  "},"} }' $1

echo "};"

echo -e "\n"

array_size=`grep -c " [BCDT] " $1`

echo "unsigned long g_drv_cmd_tbl_size = $array_size;"