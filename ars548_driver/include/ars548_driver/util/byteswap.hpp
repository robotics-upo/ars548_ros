#pragma once

/**
 * @brief Changes the endianness of the object received 
 * @tparam v The object to be modified.
 * @return T. The object modified.
 */
template<typename T>
    T byteswap(T v){
    T r;
    uint8_t *pv = (uint8_t *)&v, *pr = (uint8_t *)&r;
    for (int i = 0;i <int(sizeof(T)); i++){
        pr[i]=pv[sizeof(T)-1-i];
    }
    return r;
}