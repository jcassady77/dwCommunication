;; Auto-generated. Do not edit!


(when (boundp 'dw_listener::nodeData)
  (if (not (find-package "DW_LISTENER"))
    (make-package "DW_LISTENER"))
  (shadow 'nodeData (find-package "DW_LISTENER")))
(unless (find-package "DW_LISTENER::NODEDATA")
  (make-package "DW_LISTENER::NODEDATA"))

(in-package "ROS")
;;//! \htmlinclude nodeData.msg.html


(defclass dw_listener::nodeData
  :super ros::object
  :slots (_tagAddress _rangeNum _timeOfReception _distance _degrees _Xcoord _Ycoord _XcoordFiltered _YcoordFiltered _clockOffset _serviceData _Xaccel _Yaccel _Zaccel ))

(defmethod dw_listener::nodeData
  (:init
   (&key
    ((:tagAddress __tagAddress) "")
    ((:rangeNum __rangeNum) 0)
    ((:timeOfReception __timeOfReception) 0)
    ((:distance __distance) 0)
    ((:degrees __degrees) 0)
    ((:Xcoord __Xcoord) 0)
    ((:Ycoord __Ycoord) 0)
    ((:XcoordFiltered __XcoordFiltered) 0)
    ((:YcoordFiltered __YcoordFiltered) 0)
    ((:clockOffset __clockOffset) 0)
    ((:serviceData __serviceData) 0)
    ((:Xaccel __Xaccel) 0)
    ((:Yaccel __Yaccel) 0)
    ((:Zaccel __Zaccel) 0)
    )
   (send-super :init)
   (setq _tagAddress (string __tagAddress))
   (setq _rangeNum (round __rangeNum))
   (setq _timeOfReception (round __timeOfReception))
   (setq _distance (round __distance))
   (setq _degrees (round __degrees))
   (setq _Xcoord (round __Xcoord))
   (setq _Ycoord (round __Ycoord))
   (setq _XcoordFiltered (round __XcoordFiltered))
   (setq _YcoordFiltered (round __YcoordFiltered))
   (setq _clockOffset (round __clockOffset))
   (setq _serviceData (round __serviceData))
   (setq _Xaccel (round __Xaccel))
   (setq _Yaccel (round __Yaccel))
   (setq _Zaccel (round __Zaccel))
   self)
  (:tagAddress
   (&optional __tagAddress)
   (if __tagAddress (setq _tagAddress __tagAddress)) _tagAddress)
  (:rangeNum
   (&optional __rangeNum)
   (if __rangeNum (setq _rangeNum __rangeNum)) _rangeNum)
  (:timeOfReception
   (&optional __timeOfReception)
   (if __timeOfReception (setq _timeOfReception __timeOfReception)) _timeOfReception)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:degrees
   (&optional __degrees)
   (if __degrees (setq _degrees __degrees)) _degrees)
  (:Xcoord
   (&optional __Xcoord)
   (if __Xcoord (setq _Xcoord __Xcoord)) _Xcoord)
  (:Ycoord
   (&optional __Ycoord)
   (if __Ycoord (setq _Ycoord __Ycoord)) _Ycoord)
  (:XcoordFiltered
   (&optional __XcoordFiltered)
   (if __XcoordFiltered (setq _XcoordFiltered __XcoordFiltered)) _XcoordFiltered)
  (:YcoordFiltered
   (&optional __YcoordFiltered)
   (if __YcoordFiltered (setq _YcoordFiltered __YcoordFiltered)) _YcoordFiltered)
  (:clockOffset
   (&optional __clockOffset)
   (if __clockOffset (setq _clockOffset __clockOffset)) _clockOffset)
  (:serviceData
   (&optional __serviceData)
   (if __serviceData (setq _serviceData __serviceData)) _serviceData)
  (:Xaccel
   (&optional __Xaccel)
   (if __Xaccel (setq _Xaccel __Xaccel)) _Xaccel)
  (:Yaccel
   (&optional __Yaccel)
   (if __Yaccel (setq _Yaccel __Yaccel)) _Yaccel)
  (:Zaccel
   (&optional __Zaccel)
   (if __Zaccel (setq _Zaccel __Zaccel)) _Zaccel)
  (:serialization-length
   ()
   (+
    ;; string _tagAddress
    4 (length _tagAddress)
    ;; int64 _rangeNum
    8
    ;; int64 _timeOfReception
    8
    ;; int64 _distance
    8
    ;; int64 _degrees
    8
    ;; int64 _Xcoord
    8
    ;; int64 _Ycoord
    8
    ;; int64 _XcoordFiltered
    8
    ;; int64 _YcoordFiltered
    8
    ;; int64 _clockOffset
    8
    ;; int64 _serviceData
    8
    ;; int64 _Xaccel
    8
    ;; int64 _Yaccel
    8
    ;; int64 _Zaccel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _tagAddress
       (write-long (length _tagAddress) s) (princ _tagAddress s)
     ;; int64 _rangeNum
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _rangeNum (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _rangeNum) (= (length (_rangeNum . bv)) 2)) ;; bignum
              (write-long (ash (elt (_rangeNum . bv) 0) 0) s)
              (write-long (ash (elt (_rangeNum . bv) 1) -1) s))
             ((and (class _rangeNum) (= (length (_rangeNum . bv)) 1)) ;; big1
              (write-long (elt (_rangeNum . bv) 0) s)
              (write-long (if (>= _rangeNum 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _rangeNum s)(write-long (if (>= _rangeNum 0) 0 #xffffffff) s)))
     ;; int64 _timeOfReception
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _timeOfReception (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _timeOfReception) (= (length (_timeOfReception . bv)) 2)) ;; bignum
              (write-long (ash (elt (_timeOfReception . bv) 0) 0) s)
              (write-long (ash (elt (_timeOfReception . bv) 1) -1) s))
             ((and (class _timeOfReception) (= (length (_timeOfReception . bv)) 1)) ;; big1
              (write-long (elt (_timeOfReception . bv) 0) s)
              (write-long (if (>= _timeOfReception 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _timeOfReception s)(write-long (if (>= _timeOfReception 0) 0 #xffffffff) s)))
     ;; int64 _distance
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _distance (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _distance) (= (length (_distance . bv)) 2)) ;; bignum
              (write-long (ash (elt (_distance . bv) 0) 0) s)
              (write-long (ash (elt (_distance . bv) 1) -1) s))
             ((and (class _distance) (= (length (_distance . bv)) 1)) ;; big1
              (write-long (elt (_distance . bv) 0) s)
              (write-long (if (>= _distance 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _distance s)(write-long (if (>= _distance 0) 0 #xffffffff) s)))
     ;; int64 _degrees
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _degrees (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _degrees) (= (length (_degrees . bv)) 2)) ;; bignum
              (write-long (ash (elt (_degrees . bv) 0) 0) s)
              (write-long (ash (elt (_degrees . bv) 1) -1) s))
             ((and (class _degrees) (= (length (_degrees . bv)) 1)) ;; big1
              (write-long (elt (_degrees . bv) 0) s)
              (write-long (if (>= _degrees 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _degrees s)(write-long (if (>= _degrees 0) 0 #xffffffff) s)))
     ;; int64 _Xcoord
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _Xcoord (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _Xcoord) (= (length (_Xcoord . bv)) 2)) ;; bignum
              (write-long (ash (elt (_Xcoord . bv) 0) 0) s)
              (write-long (ash (elt (_Xcoord . bv) 1) -1) s))
             ((and (class _Xcoord) (= (length (_Xcoord . bv)) 1)) ;; big1
              (write-long (elt (_Xcoord . bv) 0) s)
              (write-long (if (>= _Xcoord 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _Xcoord s)(write-long (if (>= _Xcoord 0) 0 #xffffffff) s)))
     ;; int64 _Ycoord
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _Ycoord (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _Ycoord) (= (length (_Ycoord . bv)) 2)) ;; bignum
              (write-long (ash (elt (_Ycoord . bv) 0) 0) s)
              (write-long (ash (elt (_Ycoord . bv) 1) -1) s))
             ((and (class _Ycoord) (= (length (_Ycoord . bv)) 1)) ;; big1
              (write-long (elt (_Ycoord . bv) 0) s)
              (write-long (if (>= _Ycoord 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _Ycoord s)(write-long (if (>= _Ycoord 0) 0 #xffffffff) s)))
     ;; int64 _XcoordFiltered
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _XcoordFiltered (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _XcoordFiltered) (= (length (_XcoordFiltered . bv)) 2)) ;; bignum
              (write-long (ash (elt (_XcoordFiltered . bv) 0) 0) s)
              (write-long (ash (elt (_XcoordFiltered . bv) 1) -1) s))
             ((and (class _XcoordFiltered) (= (length (_XcoordFiltered . bv)) 1)) ;; big1
              (write-long (elt (_XcoordFiltered . bv) 0) s)
              (write-long (if (>= _XcoordFiltered 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _XcoordFiltered s)(write-long (if (>= _XcoordFiltered 0) 0 #xffffffff) s)))
     ;; int64 _YcoordFiltered
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _YcoordFiltered (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _YcoordFiltered) (= (length (_YcoordFiltered . bv)) 2)) ;; bignum
              (write-long (ash (elt (_YcoordFiltered . bv) 0) 0) s)
              (write-long (ash (elt (_YcoordFiltered . bv) 1) -1) s))
             ((and (class _YcoordFiltered) (= (length (_YcoordFiltered . bv)) 1)) ;; big1
              (write-long (elt (_YcoordFiltered . bv) 0) s)
              (write-long (if (>= _YcoordFiltered 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _YcoordFiltered s)(write-long (if (>= _YcoordFiltered 0) 0 #xffffffff) s)))
     ;; int64 _clockOffset
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _clockOffset (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _clockOffset) (= (length (_clockOffset . bv)) 2)) ;; bignum
              (write-long (ash (elt (_clockOffset . bv) 0) 0) s)
              (write-long (ash (elt (_clockOffset . bv) 1) -1) s))
             ((and (class _clockOffset) (= (length (_clockOffset . bv)) 1)) ;; big1
              (write-long (elt (_clockOffset . bv) 0) s)
              (write-long (if (>= _clockOffset 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _clockOffset s)(write-long (if (>= _clockOffset 0) 0 #xffffffff) s)))
     ;; int64 _serviceData
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _serviceData (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _serviceData) (= (length (_serviceData . bv)) 2)) ;; bignum
              (write-long (ash (elt (_serviceData . bv) 0) 0) s)
              (write-long (ash (elt (_serviceData . bv) 1) -1) s))
             ((and (class _serviceData) (= (length (_serviceData . bv)) 1)) ;; big1
              (write-long (elt (_serviceData . bv) 0) s)
              (write-long (if (>= _serviceData 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _serviceData s)(write-long (if (>= _serviceData 0) 0 #xffffffff) s)))
     ;; int64 _Xaccel
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _Xaccel (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _Xaccel) (= (length (_Xaccel . bv)) 2)) ;; bignum
              (write-long (ash (elt (_Xaccel . bv) 0) 0) s)
              (write-long (ash (elt (_Xaccel . bv) 1) -1) s))
             ((and (class _Xaccel) (= (length (_Xaccel . bv)) 1)) ;; big1
              (write-long (elt (_Xaccel . bv) 0) s)
              (write-long (if (>= _Xaccel 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _Xaccel s)(write-long (if (>= _Xaccel 0) 0 #xffffffff) s)))
     ;; int64 _Yaccel
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _Yaccel (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _Yaccel) (= (length (_Yaccel . bv)) 2)) ;; bignum
              (write-long (ash (elt (_Yaccel . bv) 0) 0) s)
              (write-long (ash (elt (_Yaccel . bv) 1) -1) s))
             ((and (class _Yaccel) (= (length (_Yaccel . bv)) 1)) ;; big1
              (write-long (elt (_Yaccel . bv) 0) s)
              (write-long (if (>= _Yaccel 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _Yaccel s)(write-long (if (>= _Yaccel 0) 0 #xffffffff) s)))
     ;; int64 _Zaccel
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _Zaccel (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _Zaccel) (= (length (_Zaccel . bv)) 2)) ;; bignum
              (write-long (ash (elt (_Zaccel . bv) 0) 0) s)
              (write-long (ash (elt (_Zaccel . bv) 1) -1) s))
             ((and (class _Zaccel) (= (length (_Zaccel . bv)) 1)) ;; big1
              (write-long (elt (_Zaccel . bv) 0) s)
              (write-long (if (>= _Zaccel 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _Zaccel s)(write-long (if (>= _Zaccel 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _tagAddress
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _tagAddress (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64 _rangeNum
#+(or :alpha :irix6 :x86_64)
      (setf _rangeNum (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _rangeNum (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _timeOfReception
#+(or :alpha :irix6 :x86_64)
      (setf _timeOfReception (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _timeOfReception (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _distance
#+(or :alpha :irix6 :x86_64)
      (setf _distance (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _distance (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _degrees
#+(or :alpha :irix6 :x86_64)
      (setf _degrees (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _degrees (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _Xcoord
#+(or :alpha :irix6 :x86_64)
      (setf _Xcoord (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _Xcoord (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _Ycoord
#+(or :alpha :irix6 :x86_64)
      (setf _Ycoord (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _Ycoord (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _XcoordFiltered
#+(or :alpha :irix6 :x86_64)
      (setf _XcoordFiltered (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _XcoordFiltered (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _YcoordFiltered
#+(or :alpha :irix6 :x86_64)
      (setf _YcoordFiltered (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _YcoordFiltered (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _clockOffset
#+(or :alpha :irix6 :x86_64)
      (setf _clockOffset (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _clockOffset (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _serviceData
#+(or :alpha :irix6 :x86_64)
      (setf _serviceData (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _serviceData (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _Xaccel
#+(or :alpha :irix6 :x86_64)
      (setf _Xaccel (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _Xaccel (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _Yaccel
#+(or :alpha :irix6 :x86_64)
      (setf _Yaccel (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _Yaccel (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _Zaccel
#+(or :alpha :irix6 :x86_64)
      (setf _Zaccel (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _Zaccel (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get dw_listener::nodeData :md5sum-) "33334d719b640f53532bf5bf4909ab5c")
(setf (get dw_listener::nodeData :datatype-) "dw_listener/nodeData")
(setf (get dw_listener::nodeData :definition-)
      "
string tagAddress
int64 rangeNum
int64 timeOfReception
int64 distance
int64 degrees
int64 Xcoord
int64 Ycoord
int64 XcoordFiltered
int64 YcoordFiltered
int64 clockOffset
int64 serviceData
int64 Xaccel
int64 Yaccel
int64 Zaccel

")



(provide :dw_listener/nodeData "33334d719b640f53532bf5bf4909ab5c")


