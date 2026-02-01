;@const-symbol-strings
;uncomment the appropriate CRC below, and appropriate counters further below. lines 6 and 30-45 for a GT, lines 8 and 47-62 for an XRC. ensure the other is commented.
;set factory def below to 1 to run init. power controller externally to keep it on, erase and stream lisp, then turn on bms. console should output a success message and bms will remain on after 30ish seconds.
;const
(def key-crc '(3141361152u32))  ;AES-128 key. Offset in 6109: 0x12009
;GT CRC
(def counter-crc '(4092889840u32))
;XRC crc
;(def counter-crc '(1860859512u32))
(def factory 0); runs factory init if set to 1

(def key (bufcreate 16))
(bufset-u8 key 0 94)  ; 5E
(bufset-u8 key 1 45)  ; 2D
(bufset-u8 key 2 98)  ; 62
(bufset-u8 key 3 103) ; 67
(bufset-u8 key 4 223) ; DF
(bufset-u8 key 5 146) ; 92
(bufset-u8 key 6 201) ; C9
(bufset-u8 key 7 182) ; B6
(bufset-u8 key 8 151) ; 97
(bufset-u8 key 9 62)  ; 3E
(bufset-u8 key 10 96) ; 60
(bufset-u8 key 11 131) ; 83
(bufset-u8 key 12 74) ; 4A
(bufset-u8 key 13 47) ; 2F
(bufset-u8 key 14 82) ; 52
(bufset-u8 key 15 236)
(def counter (bufcreate 16))
;GT Counter
(bufset-u8 counter 0 0)   ; 00
(bufset-u8 counter 1 0)   ; 00
(bufset-u8 counter 2 237) ; ED
(bufset-u8 counter 3 37)  ; 25
(bufset-u8 counter 4 247) ; F7
(bufset-u8 counter 5 65)  ; 41
(bufset-u8 counter 6 211) ; D3
(bufset-u8 counter 7 7)   ; 07
(bufset-u8 counter 8 21)  ; 15
(bufset-u8 counter 9 171) ; AB
(bufset-u8 counter 10 199) ; C7
(bufset-u8 counter 11 157) ; 9D
(bufset-u8 counter 12 246) ; F6
(bufset-u8 counter 13 35)  ; 23
(bufset-u8 counter 14 116) ; 74
(bufset-u8 counter 15 72) ; 48
;XRC counter
;(bufset-u8 counter 0 0)   ; 00
;(bufset-u8 counter 1 26)  ; 1A
;(bufset-u8 counter 2 8)   ; 08
;(bufset-u8 counter 3 116) ; 74
;(bufset-u8 counter 4 88)  ; 58
;(bufset-u8 counter 5 214) ; D6
;(bufset-u8 counter 6 154) ; 9A
;(bufset-u8 counter 7 127) ; 7F
;(bufset-u8 counter 8 76)  ; 4C
;(bufset-u8 counter 9 24)  ; 18
;(bufset-u8 counter 10 39) ; 27
;(bufset-u8 counter 11 248) ; F8
;(bufset-u8 counter 12 230) ; E6
;(bufset-u8 counter 13 135) ; 87
;(bufset-u8 counter 14 169) ; A9
;(bufset-u8 counter 15 193) ; C1

(def magic [0xff 0x55 0x00])

@const-start
;utils
(defun max (a b)
    (if (> a b) a b)
)

(defun min (a b)
    (if (< a b) a b)
)

(defun print-hex (data)
    (print
        (map (fn (x) (bufget-u8 data x)) (range (buflen data)))
    )
)

(defun event-handler ()
    (loopwhile t
        (recv
            ((event-esp-now-rx (? src) (? des) (? data) (? rssi)) (pubmote-rx src des data rssi))
            ((event-data-rx . (? data)) (float-command-rx data))
            (_ nil)
        )
    )
)

(defun send-msg (text)
    (send-data (str-merge "msg " text))
)

(defun send-status (text)
    (send-data (str-merge "status " text))
)

(defun mklist (len val)
    (map (fn (x) val) (range len))
)

(defun split-list (lst n)
    (if (eq lst nil)
        nil
        (cons (take lst n) (split-list (drop lst n) n))
    )
)

(defunret pack-bytes-to-uint32 (byte-list) {
  (return (to-u32 (+ (shl (to-u32 (ix byte-list 0)) 24)
                     (shl (to-u32 (ix byte-list 1)) 16)
                     (shl (to-u32 (ix byte-list 2)) 8)
                     (to-u32 (ix byte-list 3)))))
})
(defunret unpack-uint32-to-bytes (packed-value) {
  (return (list (to-byte (shr packed-value 24))
                (to-byte (shr (bitwise-and packed-value 0xFF0000) 16))
                (to-byte (shr (bitwise-and packed-value 0xFF00) 8))
                (to-byte (bitwise-and packed-value 0xFF))))
})


(defun estimate-soc (v voltage-curve) {
    (var n (length voltage-curve))
    (var socs (list 100 90 80 70 60 50 40 30 20 10 0))
    (cond
        ((>= v (ix voltage-curve 0)) 100.0)
        ((<= v (ix voltage-curve (- n 1))) 0.0)
        (true
            (looprange i 1 (- n 1)
                (if (and (>= v (ix voltage-curve i)) (<= v (ix voltage-curve (- i 1))))
                    (break (let ((v1 (ix voltage-curve (- i 1)))
                    (v2 (ix voltage-curve i))
                    (s1 (ix socs (- i 1)))
                    (s2 (ix socs i)))
                    (+ s1 (* (/ (- v v1) (- v2 v1)) (- s2 s1)))))
                )
            )
         )
     )
})


;bms lisp


;aes

(def bms-user-cmd -1)
(def bms-loop-delay 8)
;(def bms-rs485-di-pin) Uart is on the com port
;(def bms-rs485-ro-pin)
;(def `pin_adc3 `pin_adc3)
;(def bms-wakeup-pin)
(def bms-use-crypto 1)
(def bms-override-soc 0)
(def bms-type 1)
(def bms-rs485-chip 1)
(def bms-charge-only 0)
(def bms-buff-size 128)
(def bms-last-activity-time)


;vars
(def cell-count-uninit t)
(def is-charging -1)
(def is-current-over-limit -1)
(def is-battery-empty -1)
(def is-battery-temp-out-of-range -1)
(def is-battery-overcharged -1)
(def serial -1)
(def bms-status -1)
(def bms-battery-type -1)
(def bms-battery-cycles -1)
(def bms-charge-state 0)

(defun crypt (nonce-high nonce-low data start-offset len) {
    (var restore-byte (bufget-u8 counter 15))
    (bufset-u8 counter 0 nonce-high)
    (bufset-u8 counter 1 nonce-low)
    (aes-ctr-crypt key counter data start-offset len)
    (bufset-u8 counter 15 restore-byte)
})

(defunret checksum (data start-offset len) {
    (var sum 0)
    (looprange k start-offset len {
        (setq sum (+ sum (bufget-u8 data k)))
    })
    (return sum)
})

(defunret set-charge-state (nonce-high nonce-low charge-state) {
    (if (>= bms-charge-state 0){
        (def payload (bufcreate 2))
        (bufset-u8 payload 0 0x64)
        (bufset-u8 payload 1 (if (= charge-state 0x1) 0x00 0x01))
        (def packet (construct-packet nonce-high nonce-low payload))
        (free payload)
        (gpio-write 'pin-adc3 1)
        (uart-write packet)
        (sleep 0.005)
        (gpio-write 'pin-adc3 0)
        (sleep 0.005)
        (var ret (recv-packets 0x15 -1))
        (free packet)
        (return ret)
    }{
        (print "Haven't received current bms-charge-state yet")
        (return false)
    })
})

(defunret factory-init-accept () {
    (var init-payload2 (bufcreate 2))
    (bufset-u8 init-payload2 0 0x03)
    (bufset-u8 init-payload2 1 0x01)
    (def init-packet2 (construct-packet 0x00 0x01 init-payload2))
    (free init-payload2)
    (gpio-write 'pin-adc3 1)
    (uart-write init-packet2)
    (sleep 0.005)
    (gpio-write 'pin-adc3 0)
    (sleep 0.005)
    (var ret (recv-packets 0x0e 1))
    (free init-packet2)
    (return ret)
})

(defunret factory-init () {
    (var init-payload (bufcreate 4))
    (bufset-u8 init-payload 0 0x05)
    (bufset-u8 init-payload 1 0x52)
    (bufset-u8 init-payload 2 0x4d)
    (bufset-u8 init-payload 3 0x41);05RMA
    (def init-packet (construct-packet 0x00 0x01 init-payload))
    (free init-payload)
    (gpio-write 'pin-adc3 1)
    (uart-write init-packet)
    (sleep 0.005)
    (gpio-write 'pin-adc3 0)
    (sleep 0.005)
    (var ret (recv-packets 0x0e 0))
    (free init-packet)
    (return ret)
})
(defunret process-packet (data) {
    (if (!=(bufget-u8 data 2) (bufget-u8 magic 2)){
        (return false)
    })
    (var packet-checksum (bufget-u16 data (- (buflen data) 2)))
    (var calc-checksum (checksum data 0 (- (buflen data) 2)))
    (if (not-eq packet-checksum calc-checksum){
        (return false)
    })
    (if bms-use-crypto {
        (return (crypt (bufget-u8 data 3) (bufget-u8 data 4) data 5 (- (buflen data) 7)))
    })
    (return true)
})

(defunret construct-packet (nonce-high nonce-low payload) {
    (var packet (bufcreate (+ (buflen payload) 7)))
    (bufcpy packet 0 magic 0 (buflen magic))
    (bufset-u8 packet 3 nonce-high)
    (bufset-u8 packet 4 nonce-low)
    (crypt nonce-high nonce-low payload 0 (buflen payload))
    (bufcpy packet 5 payload 0 (buflen payload))
    (var calc-checksum  (checksum packet 0 (- (buflen packet) 2)))
    (bufset-u8 packet (+ (buflen payload) 5) (shr calc-checksum 8))
    (bufset-u8 packet (+ (buflen payload) 6) (bitwise-and calc-checksum 0xFF))
    (return packet)
})

(defun parse-voltage (data){
    ;(set-bms-val 'bms-v-tot (/ (bufget-u8 data (if bms-use-crypto 6 4)) 100.0))
    ;(print data)
})

(defun parse-cell-voltage (data){
    (var cell-index 0)
    (var total-voltage 0)
    (var v-cell-min (/ (bufget-u16 data (if bms-use-crypto 6 4)) (if bms-use-crypto 10000.0 1000.0)))
    (var v-cell-max v-cell-min)
    (if cell-count-uninit {
        (set-bms-val 'bms-cell-num (/ (- (buflen data) 8) 2))
        (setq cell-count-uninit false)
    })

    (looprange k (if bms-use-crypto 6 4) (- (buflen data) (+ (if bms-use-crypto 0 2) 3)) { ;Need to leave off end 16th cell for 15s BMS
        (if (eq (mod k 2) 0) {
            (var current-cell (/ (bufget-u16 data k) (if bms-use-crypto 10000.0 1000.0)))
            (if (and (= cell-index 0) (= bms-override-soc 1)) {
                (set-bms-val 'bms-soc (/ (estimate-soc current-cell voltage-curve) 100))
            })
            (set-bms-val 'bms-v-cell cell-index current-cell)
            (setq cell-index (+ cell-index 1))
            (setq total-voltage (+ total-voltage current-cell))
            (if (> current-cell v-cell-max) (setq v-cell-max current-cell))
            (if (< current-cell v-cell-min) (setq v-cell-min current-cell))
        })
    })
    (set-bms-val 'bms-v-tot total-voltage)
    (set-bms-val 'bms-v-cell-min v-cell-min)
    (set-bms-val 'bms-v-cell-max v-cell-max)

})

(defun parse-soc (data){
    (if (= bms-override-soc 0) {
        (var soc (/ (bufget-u8 data (if bms-use-crypto 6 4)) 100.0))
        (if (> soc 1.0) (setq soc 1.0))
        (if (< soc 0.01) (setq soc 0.01))
        (set-bms-val 'bms-soc soc)
    })
})

(defun parse-current (data) {
    (var current-scaler (if bms-use-crypto 0.0366 0.055)); Current scaler. 0.0366 or 0.0378 not sure...
    ;maybe look at tot-current from controller and then calculate scaler and take average to see which is closer?
    (var current-limit (if bms-use-crypto 32.0 30.0)) ;Limit 32A or 32.7A, not sure.
    (var current (* (bufget-i16 data (if bms-use-crypto 6 4)) current-scaler))

    (if (and (not bms-charge-only) (>= current current-limit)) {
        (setq is-current-over-limit 1)
    }{
        (setq is-current-over-limit 0)
    })
    (set-bms-val 'bms-i-in-ic current)
})

(defun parse-status (data) {
    (setq bms-status (bufget-u8 data (if bms-use-crypto 6 4)))
    (setq is-charging (if (= (bitwise-and bms-status 0x20) 0) 0 1))
    (setq is-battery-empty (if (= (bitwise-and bms-status 0x4) 0) 0 1))
    (setq is-battery-temp-out-of-range (if (= (bitwise-and bms-status 0x3) 0) 0 1))
    (setq is-battery-overcharged (if (= (bitwise-and bms-status 0x8) 0) 0 1))
    (var is-soc-calculating (if (= (bitwise-and bms-status 0x40) 0) 0 1)) ;maybe or could be if balancing? Not sure cuz it came on while riding
    ;well also observed -128 but have no clue what that means
    (if (= is-charging 1) (if (<= (get-bms-val 'bms-i-in-ic) -0.5) (set-bms-val 'bms-v-charge (get-bms-val 'bms-v-tot) ) ) )
})

(defun parse-temp (data) {
    (set-bms-val 'bms-temp-ic (bufget-i8 data (- (buflen data) 3)))
    (var t-cell-max (bufget-i8 data (if bms-use-crypto 6 4)))
    (looprange k (if bms-use-crypto 6 4) (- (buflen data) 3) {
        (var temp-val (bufget-i8 data k))
        (if (> temp-val t-cell-max) (setq t-cell-max temp-val))
        (set-bms-val 'bms-temps-adc (- k (if bms-use-crypto 6 4)) temp-val)
    })
    (set-bms-val 'bms-temp-cell-max t-cell-max)
})

(defun parse-serial (data) {
    (setq serial (bufget-u16 data (if bms-use-crypto 6 4)))
})

(defun parse-charger (data) {
    (setq bms-charge-state (bufget-u8 data (if bms-use-crypto 6 4)))
})

(defun parse-battery-type (data) {
    ;(var battery-types '((UNDEFINED . 0) (A123_LiFePO4 . 1) (VTC6 . 1) (HG2 . 3) (30Q . 4) (VTC5A . 5) (VTC5D . 6) (30Q6 . 7) (P28A . 8) (VTC6A . 9) (P42A . 10) (40T3 . 11) ))
    (setq bms-battery-type (bufget-u8 data (if bms-use-crypto 6 4)))
})

(defun parse-cycles-health (data) {
    (setq bms-battery-cycles (bufget-u16 data (if bms-use-crypto 6 4)))
    ;TODO No way to set SoH in Lisp?
    (var soh (/ (bufget-u8 data (if bms-use-crypto 8 6)) 100.0))
    (if (> soh 1.0) (setq soh 1.0))
    (if (< soh 0.01) (setq soh 0.01))
    (set-bms-val 'bms-soh soh)
})

(defunret process-cmd (command data ack handshake) {
    (cond
        ((= command 0x00) {
            (parse-status data)
        })
        ;((= command 0x01) {
            ;(parse-voltage data);donno why this doesn't get emmited sometimes
        ;})
        ((= command 0x02) {
            (parse-cell-voltage data)
        })
        ((= command 0x03) {
            (parse-soc data)
        })
        ((= command 0x04) {
            (parse-temp data)
        })
        ((= command 0x05) {
            (parse-current data)
        })
        ((= command 0x06) {
            (parse-serial data)
        })
        ;((= command 0x07) { ; acknowleged as valid packet by controller but no processing ;Emitted by BMS
            ;static maybe these next bytes have to do with firmware build or hardware?
        ;})
        ((= command 0x08) {
            (parse-battery-type data)
        })
        ;((= command 0x09) { ;looked at by controller
        ;})
        ;((= command 0x0A) { ; acknowleged as valid packet by controller but no processing. Sent during bms startup?
        ;})
        ;((= command 0x0B) { ;Emitted by BMS
            ;static
        ;})
        ;((= command 0x0C) { ; acknowleged as valid packet by controller but no processing ;Emitted by BMS. Seems to be all 0 unless on charger.
        ;})
        ((= command 0x0D) {
            (parse-cycles-health data)
        })
        ((= command 0x0E) {
            (if ack {
                (if (= handshake 0) {
                    (return (factory-init-accept))
                })
                (if (= handshake 1) {
                    (var val (bufget-u16 data (if bms-use-crypto 6 4)))
                    (if (!= val 1) { (return false)})
                    (send-msg "Success")
                    (return true)
                })

            })
        })
        ;((= command 0x0F) { ; Looked at by controller ;Emitted by BMS
        ;    ;error?
        ;})
        ;((= command 0x10) { ; looked at by controller ;Emitted by BMS
        ;    ;unknown maybe error code logs?
        ;})
        ;((= command 0x11) { ; looked at by controller
        ;})
        ;((= command 0x12) { ; acknowleged as valid packet by controller but no processing ;Emitted by BMS
        ;})
        ;((= command 0x13) { ;Emitted by BMS
        ;})
        ;((= command 0x14) { ;Emitted by BMS
        ;})
        ((= command 0x15) {
            (if (and ack (= (bufget-u8 data (if bms-use-crypto 6 4)) bms-charge-state)) {(parse-charger data) (return false)})
            (parse-charger data)
        })
        ;((= command 0x16) { ; looked at by controller ;Emitted by BMS
            ;unknown last byte changes
        ;})
        ;So in bms recv we know there's no valid commands above 0x16 (command 0x64 is special and gets sent from controller to BMS)
        ;((= command 0x64) { ;Sent by controller. Should be recieved by BMS and update 0x15
            ;bms-charge-state cmd
            ;(print "charge state cmd")
        ;})
        (t {
            ;(print (str-from-n command "Unknown Command: 0x%0x"))
            (return false)
        })
        )
    (return true)
})

;(defun load-keys () {
;    (var key-list (append (unpack-uint32-to-bytes (get-config 'bms-key-a)) (unpack-uint32-to-bytes (get-config 'bms-key-b)) (unpack-uint32-to-bytes (get-config 'bms-key-c)) (unpack-uint32-to-bytes (get-config 'bms-key-d))))
;    (var counter-list (append (unpack-uint32-to-bytes (get-config 'bms-counter-a)) (unpack-uint32-to-bytes (get-config 'bms-counter-b)) (unpack-uint32-to-bytes (get-config 'bms-counter-c)) (unpack-uint32-to-bytes (get-config 'bms-counter-d))))
;    (looprange i 0  (length key-list){
;        (bufset-u8 key i (ix key-list i))
;    })
;    (looprange i 0  (length counter-list){
;        (bufset-u8 counter i (ix counter-list i))
;    })
;})

(defunret verify-keys (key counter key-crc counter-crc) {
    (var key-crc-calc (crc32 key 0))
    (var counter-crc-calc (crc32 counter 0))
    (looprange i 0 (length key-crc) {
        (if (and (= (ix key-crc i) key-crc-calc) (= (ix counter-crc i) counter-crc-calc)) (return true))
    })
    (print key)
    (return false)
})

(defunret recv-packets (cmd-ack handshake) {
    (var read-timeout (if (= cmd-ack 0x0e)
        1.0  ; 1 second for factory init
        0.5)) ; 500ms for other commands like charge state
    (var bytes-read (uart-read bms-buf (buflen bms-buf) nil nil read-timeout))
    (print bms-buf)
    (var found-packet nil)
    (var start 0)
    (loopwhile (>= (- bytes-read start) (if bms-use-crypto 12 10)) {  ; Check against minimum packet size
        ; Look for magic bytes
        (if (and (= (bufget-u8 bms-buf start) (bufget-u8 magic 0))
                 (= (bufget-u8 bms-buf (+ start 1)) (bufget-u8 magic 1))
                 (= (bufget-u8 bms-buf (+ start 2)) (bufget-u8 magic 2))) {
            (setq bms-last-activity-time (systime)) ; Update after we see magic
            (setq found-packet t)
            ; Find the end of the packet (next magic bytes or end of buffer)
            (var packet-end start)
            (looprange j (+ start 3) (- bytes-read 2) {
                (if (and (= (bufget-u8 bms-buf j) (bufget-u8 magic 0))
                         (= (bufget-u8 bms-buf (+ j 1)) (bufget-u8 magic 1))
                         (= (bufget-u8 bms-buf (+ j 2)) (bufget-u8 magic 2))) {
                    (setq packet-end j)
                    (break)
                })
            })
            (if (= packet-end start) {
                (setq packet-end bytes-read) ; If no end found, process to end of buffer
            })

            ; Create a new buffer for the packet
            (var packet-length (- packet-end start))
            (var packet (bufcreate packet-length))
            (bufcpy packet 0 bms-buf start packet-length)
            ; Process the packet
            (if (process-packet packet) {
                (var command (bufget-u8 packet (if bms-use-crypto 5 3))) ; Adjust for crypto/non-crypto

                (print (str-from-n command "Command: %0x"))
                (if (and (>= cmd-ack 0) (= command cmd-ack)) {
                    (var result (process-cmd command packet t handshake))
                    (free packet)
                    (return result)
                })
                (process-cmd command packet nil -1)
            })
            (free packet)
            (setq start packet-end) ; Move start to the end of the processed packet
        } {
            (setq start (+ start 1)) ; Move to the next byte if magic bytes not found
        })
    })
    (if found-packet (send-bms-can))
    (return (< cmd-ack 0));if we're looking for an ack we failed to find one
})

(defunret init-bms () {
    (uart-stop)
    (sleep 1)
    (if bms-use-crypto{

        ;(load-keys)
        (if (eq (verify-keys key counter key-crc counter-crc) false){
            (send-msg "Invalid keys")
            (return false)
        } {

        })
    })
    (gpio-configure 'pin-adc3 'pin-mode-out)
    (gpio-write 'pin-adc3 0)
    (uart-start 115200);If GNSS is connected UART 1 must be used
    (set-bms-val 'bms-temp-adc-num 4)
    (bufset-u8 magic 2 (if bms-use-crypto 0xbb 0xaa))
    (yield 100000);Gotta make sure uart is ready
    ;now we're cookin'
    (return true)
})

; (defun load-bms-settings (){
;     (setq bms-override-soc (get-config 'bms-override-soc))
;     (setq bms-type (get-config 'bms-type))
;     (setq bms-use-crypto nil)
;     (if (> bms-type 1) (setq bms-use-crypto t))
;     (setq bms-rs485-chip (get-config 'bms-rs485-chip))
;     (setq bms-loop-delay (get-config 'bms-loop-delay))
;     (setq bms-charge-only (get-config 'bms-charge-only))
;})

(defunret bms-loop () {
    ;(load-bms-settings)
    (if (and (> bms-type 0) (init-bms)) {

        (var next-run-time (secs-since 0))  ; Set first run time
        (var loop-start-time 0)
        (var loop-end-time 0)
        (def bms-buf (bufcreate bms-buff-size))
        (var bms-loop-delay-sec (/ 1.0 bms-loop-delay))
        (setq bms-last-activity-time 0)
        (loopwhile t{
        (setq loop-start-time (secs-since 0))
            ;lets check if there's a user command
            (if (!= bms-user-cmd -1) {
                 (if (and (= bms-rs485-chip 1) bms-use-crypto (< (secs-since bms-last-activity-time) 1)) {
                    (loopwhile (not (if (= bms-user-cmd 0x0e) (factory-init) (set-charge-state 0x00 0x01 bms-charge-state))) {(sleep 0.01)})
                 }{
                    ;error
                    (print "BMS not connected")
                 })
                 (setq bms-user-cmd -1)
            }{
                (var prev-activity bms-last-activity-time)
                (recv-packets -1 -1)
                (if (and (= factory 1) (!= prev-activity bms-last-activity-time)) {
                    (if (factory-init) {
                        (print "Factory Init Success")
                        (setq factory 0)
                    })
                })
            })

            (setq loop-end-time (secs-since 0))
            (var actual-loop-time (- loop-end-time loop-start-time))

            (var time-to-wait (- next-run-time (secs-since 0)))
            (if (> time-to-wait 0) {
                (yield (* time-to-wait 1000000))
            }{
                (setq next-run-time (secs-since 0))
            })
            (setq next-run-time (+ next-run-time bms-loop-delay-sec))
        })
        (free bms-buf)
        (setq bms-exit-flag nil)
    })
})

(spawn bms-loop)
@const-end