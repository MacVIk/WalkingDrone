/*
 * Terminal.h
 *
 *  Created on: 15 ���. 2020 �.
 *      Author: Taras.Melnik
 */

#ifndef CODE_TERMINAL_H_
#define CODE_TERMINAL_H_

#include "TaskWrapper.h"

class Terminal: public TaskWrapper {
public:
    Terminal();
    virtual ~Terminal();

    void run() override;
};

extern Terminal terminal;

#endif /* CODE_TERMINAL_H_ */
