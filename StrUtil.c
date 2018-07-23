// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     StrUtil.C
//
//   String handling, useful variations on standard C string library.
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//

// Alternative to standard C lib strcpy( )
// copies source string to dest, returns char pointer to trailing null
// of resulting string, which can then be used as dest for subsequent call
// when you want to string several things together
char* strU_strcpy(char* dest,const char* source){
	char* d_ptr = dest;
	const char* s_ptr = source;
	while (*s_ptr) {
		*(d_ptr++) = *(s_ptr++);
	}
	return d_ptr;
}
