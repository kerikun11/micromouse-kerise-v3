/**
    @file TaskBase.h
    @brief FreeRTOSのTaskにC++の関数を渡すためのクラスを定義したファイル．
    @author KERI (Github: kerikun11)
    @date 2017.11.27
*/
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/** @class TaskBase
    @brief FreeRTOSのタスクのベースとなるクラス．
    実行したい関数をもつクラスでこのクラスを継承して使用する．
*/
class TaskBase {
  public:
    /** @function Constructor
        このクラス単体を宣言することはないだろう
    */
    TaskBase(): pxCreatedTask(NULL) {}
    /** @function Destructor
        もしタスクが実行中なら削除する
    */
    ~TaskBase() {
      deleteTask();
    }
    /** @function createTask
        @brief タスクを生成する関数
    */
    bool createTask(const char* pcName, UBaseType_t uxPriority = 0, const uint16_t usStackDepth = configMINIMAL_STACK_SIZE, const BaseType_t xCoreID = tskNO_AFFINITY) {
      if (pxCreatedTask != NULL) {
        log_w("task %s is already created", pcName);
        return false;
      }
      // Taskを生成
      BaseType_t res = xTaskCreatePinnedToCore(pxTaskCode, pcName, usStackDepth, this, uxPriority, pxCreatedTask, xCoreID);
      if (res != pdPASS) {
        log_w("couldn't create the task %s", pcName);
        return false;
      }
      return true;
    }
    /** @function deleteTask
        @brief タスクを削除する関数
    */
    void deleteTask() {
      if (pxCreatedTask == NULL) {
        log_w("task is not created");
        return;
      }
      vTaskDelete(pxCreatedTask);
    }

  protected:
    TaskHandle_t* pxCreatedTask;  //< タスクのハンドル

    /** @function task
        @brief FreeRTOSにより実行される関数名
    */
    virtual void task() = 0;
    /** @function task
        @brief FreeRTOSにより実行される関数ポインタ
    */
    static void pxTaskCode(void* const pvParameters) {
      static_cast<TaskBase*>(pvParameters)->task();
    }
};

