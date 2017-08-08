import cv2
import time
fourcc = cv2.VideoWriter_fourcc(*'XVID')
wf = 848
hf = 480
out = cv2.VideoWriter('/home/infosys/IMG_1613_out.avi',fourcc, 5.0, (wf,hf))
with detection_graph.as_default():
    with tf.Session(graph=detection_graph) as sess:
        counter = 0
        wf = 848
        hf = 480
        

        for image_path in TEST_IMAGE_PATHS:
            
            if (counter % 2 == 0):
                image = cv2.imread(image_path)
                image_np = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) 
                # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                image_np_expanded = np.expand_dims(image_np, axis=0)
                start_time = time.time()
                image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
                # Each box represents a part of the image where a particular object was detected.
                boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                scores = detection_graph.get_tensor_by_name('detection_scores:0')
                classes = detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = detection_graph.get_tensor_by_name('num_detections:0')
                
                # Actual detection.
                (boxes, scores, classes, num_detections) = sess.run(
                  [boxes, scores, classes, num_detections],
                  feed_dict={image_tensor: image_np_expanded})
                print("--- %s seconds ---" % (time.time() - start_time))
                # Visualization of the results of a detection.
                vis_util.visualize_boxes_and_labels_on_image_array(
                  image_np,
                  np.squeeze(boxes),
                  np.squeeze(classes).astype(np.int32),
                  np.squeeze(scores),
                  category_index,
                  use_normalized_coordinates=True,
                  line_thickness=6)
                
                plt.figure(figsize=IMAGE_SIZE)
                plt.imshow(image_np)
                try:
                    out.write(image_np)
                except:
                    print ("Error: write")
                cv2.waitKey(1)

        
    #out.write(image_np)
    out.release()
    plt.show()
