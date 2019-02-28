import boto3 # aws
import os
import io
import cv2
from PIL import Image
rekognition = boto3.client('rekognition', region_name='us-west-2')
dynamodb = boto3.client('dynamodb', region_name='us-west-2')

class Face(object):
    def __init__(self):
        pass
    def face_match(self,target):
        arr = os.listdir(os.getcwd()+"/AWS/source-image")
        label = None
        confidence = None
        for file in arr:
            print (file)
            try:
                sourceFile = os.getcwd() + '/AWS/source-image/'+file
                targetFile = target
                client = boto3.client('rekognition')
                imageSource = open(sourceFile, 'rb')
                imageTarget = open(targetFile, 'rb')

                response = client.compare_faces(SimilarityThreshold=70,
                                                SourceImage={'Bytes': imageSource.read()},
                                                TargetImage={'Bytes': imageTarget.read()})

                for faceMatch in response['FaceMatches']:
                    position = faceMatch['Face']['BoundingBox']
                    confidence = str(faceMatch['Face']['Confidence'])
                    # print('The face at ' +
                    #       str(position['Left']) + ' ' +
                    #       str(position['Top']) +
                    #       ' matches with ' + confidence + '% confidence') + ' name '+ str(file)
                    label = str(file)
                    confidence = confidence
                imageSource.close()
                imageTarget.close()
            except Exception as error:
                label = None
                confidence = None
        return label, confidence
    def detect_label(self, image_path):
        client = boto3.client('rekognition')
        imageSource=open(image_path,'rb')
        imageByte = {'Bytes':imageSource}
        label_detect = client.detect_labels(Image=imageByte)
        print (label_detect)
    def face_s3_match(self, target_image):
        targetFile = target_image
        imageTarget = open(targetFile, 'rb')
        response = rekognition.search_faces_by_image(
            CollectionId='family_collection',
            Image={'Bytes': imageTarget.read()}
        )

        for match in response['FaceMatches']:
            print(match['Face']['FaceId'], match['Face']['Confidence'])

            face = dynamodb.get_item(
                TableName='family_collection',
                Key={'RekognitionId': {'S': match['Face']['FaceId']}}
            )

            if 'Item' in face:
                print(face['Item']['FullName']['S'])
            else:
                print('no match found in person lookup')
    def face_s3_match_multi(self,file_path):
        image = Image.open(file_path)
        stream = io.BytesIO()
        image.save(stream, format="JPEG")
        image_binary = stream.getvalue()

        response = rekognition.detect_faces(
            Image={'Bytes': image_binary}
        )
        #print (response)

        all_faces = response['FaceDetails']

        # Initialize list object
        boxes = []
        label = []
        # Get image diameters
        image_width = image.size[0]
        image_height = image.size[1]

        # Crop face from image
        for face in all_faces:
            box = face['BoundingBox']
            x1 = int(box['Left'] * image_width) * 0.9
            y1 = int(box['Top'] * image_height) * 0.9
            x2 = int(box['Left'] * image_width + box['Width'] * image_width) * 1.10
            y2 = int(box['Top'] * image_height + box['Height'] * image_height) * 1.10
            image_crop = image.crop((x1, y1, x2, y2))

            stream = io.BytesIO()
            image_crop.save(stream, format="JPEG")
            image_crop_binary = stream.getvalue()

            # Submit individually cropped image to Amazon Rekognition
            try:
                response = rekognition.search_faces_by_image(
                    CollectionId='family_collection',
                    Image={'Bytes': image_crop_binary}
                )

                if len(response['FaceMatches']) > 0:
                    # Return results
                    print('Coordinates ', box)
                    for match in response['FaceMatches']:

                        face = dynamodb.get_item(
                            TableName='family_collection',
                            Key={'RekognitionId': {'S': match['Face']['FaceId']}}
                        )
                        if 'Item' in face:
                            person = face['Item']['FullName']['S']
                            label.append(face['Item']['FullName']['S'])
                        else:
                            person = 'no match found'

                        print(match['Face']['FaceId'], match['Face']['Confidence'], person)
            except Exception as error:
                pass
                #print ("Print "+error)

        #os.system("sudo rm -R " + file_path)
        return label
    def face_s3_match_multi_moduler(self,file_path):
        image = Image.open(file_path)
        stream = io.BytesIO()
        image.save(stream, format="JPEG")
        image_binary = stream.getvalue()

        response = rekognition.detect_faces(
            Image={'Bytes': image_binary}
        )
        print (response)

        all_faces = response['FaceDetails']

        # Initialize list object
        boxes = []
        label = []
        # Get image diameters
        image_width = image.size[0]
        image_height = image.size[1]

        # Crop face from image
        for face in all_faces:
            box = face['BoundingBox']
            x1 = int(box['Left'] * image_width) * 0.9
            y1 = int(box['Top'] * image_height) * 0.9
            x2 = int(box['Left'] * image_width + box['Width'] * image_width) * 1.10
            y2 = int(box['Top'] * image_height + box['Height'] * image_height) * 1.10
            image_crop = image.crop((x1, y1, x2, y2))

            stream = io.BytesIO()
            image_crop.save(stream, format="JPEG")
            image_crop_binary = stream.getvalue()

            # Submit individually cropped image to Amazon Rekognition
            try:
                response = rekognition.search_faces_by_image(
                    CollectionId='family_collection',
                    Image={'Bytes': image_crop_binary}
                )

                if len(response['FaceMatches']) > 0:
                    # Return results
                    print('Coordinates ', box)
                    for match in response['FaceMatches']:

                        face = dynamodb.get_item(
                            TableName='family_collection',
                            Key={'RekognitionId': {'S': match['Face']['FaceId']}}
                        )
                        if 'Item' in face:
                            person = face['Item']['FullName']['S']
                            label.append(face['Item']['FullName']['S'])
                        else:
                            person = 'no match found'

                        print(match['Face']['FaceId'], match['Face']['Confidence'], person)
            except Exception as error:
                pass
                #print ("Print "+error)

        #os.system("sudo rm -R " + file_path)
        return label