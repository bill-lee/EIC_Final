/* This header was created by Rong Siou Lee @ NTU BIME 2013/12/06*/
/* This file is to define the return value which is helpful at the condition
   debugging about the function
   return value > 0 represents the program is ok, with value < 0 its problem
   can be listed
*/

enum{
    RS_ok = 1,
    RS_para_1 = -1, // problem with first parameter passed
    RS_para_2 = -2, // problem with second parameter passed
    RS_para_3 = -3, // problem with third parameter passed
    RS_para_4 = -4, // problem with fourth parameter passed
    RS_FileNotFound = -5, // problem with file was not found
    RS_DeviceNotOpened = -6  // problem with device was not opened

};
