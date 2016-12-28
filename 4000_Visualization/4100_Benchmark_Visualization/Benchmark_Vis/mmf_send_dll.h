#pragma once
#ifdef __cplusplus
extern "C" {
#endif

extern void __declspec(dllexport) __stdcall mmf_init(void);
extern void __declspec(dllexport) __stdcall mmf_send(float Values[100]);
extern void __declspec(dllexport) __stdcall mmf_close(void);

#ifdef __cplusplus
}
#endif