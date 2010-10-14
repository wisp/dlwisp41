#ifndef RFID_COMMANDS_H
#define RFID_COMMANDS_H

#include "dlwisp41.h"
#include "rfid.h"

inline void handle_query (volatile short nextState);
inline void handle_queryrep (volatile short nextState);
inline void handle_queryadjust (volatile short nextState);
inline void handle_select (volatile short nextState);
inline void handle_ack (volatile short nextState);
inline void handle_request_rn (volatile short nextState);
inline void handle_read (volatile short nextState);
inline void handle_nak (volatile short nextState);
inline void do_nothing ();

#endif // RFID_COMMANDS_H
