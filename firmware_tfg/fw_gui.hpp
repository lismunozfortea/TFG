#if !defined(FW_GUI_H)
#define FW_GUI_H

enum op_status_t {OS_NORMAL, OS_BADSNSR, OS_OVRCURRENT, OS_OVRHEATED};

void gui_begin(void);
void gui_update(void);
void gui_notify(op_status_t cause);

#endif /* !defined(FW_GUI_H) */
